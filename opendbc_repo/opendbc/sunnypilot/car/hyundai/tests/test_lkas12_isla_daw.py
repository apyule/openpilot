import unittest

from opendbc.can import CANPacker, CANParser
from opendbc.car import Bus, gen_empty_fingerprint, structs
from opendbc.car.hyundai import hyundaican
from opendbc.car.hyundai.carcontroller import CarController
from opendbc.car.hyundai.carstate import CarState
from opendbc.car.hyundai.interface import CarInterface
from opendbc.car.hyundai.values import CAR
from opendbc.sunnypilot.car.hyundai.values import HyundaiFlagsSP, HyundaiSafetyFlagsSP


class TestHyundaiLkas12IslaDaw(unittest.TestCase):
  def setUp(self):
    self.packer = CANPacker("hyundai_can_generated")
    self.cam_parser = CANParser("hyundai_can_generated", [("LKAS11", 100), ("LKAS12", float("nan"))], 2)
    self.pt_parser = CANParser("hyundai_can_generated", [("LKAS11", 100), ("LKAS12", float("nan"))], 0)

  def _parse_cam(self, lkas11_hex: str, lkas12_hex: str):
    self.cam_parser.update([(1_000_000, [
      (0x340, bytes.fromhex(lkas11_hex), 2),
      (0x53E, bytes.fromhex(lkas12_hex), 2),
    ])])
    return dict(self.cam_parser.vl["LKAS11"]), dict(self.cam_parser.vl["LKAS12"])

  def test_interface_sets_isla_and_daw_flags(self):
    fp = gen_empty_fingerprint()
    fp[2][0x53E] = 8
    CP = CarInterface.get_params(CAR.HYUNDAI_ELANTRA_HEV_2024, fp, [], False, False, False)
    CP_SP = CarInterface._get_params_sp(CP, structs.CarParamsSP(), CAR.HYUNDAI_ELANTRA_HEV_2024, fp, [], False, False, False)
    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.HAS_LKAS12)
    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.MUTE_ISLA_LKAS12)
    self.assertTrue(CP_SP.flags & HyundaiFlagsSP.SUPPRESS_DAW_LKAS12)
    self.assertTrue(CP_SP.safetyParam & HyundaiSafetyFlagsSP.MUTE_ISLA_LKAS12)

    # 6-byte legacy LKAS12 should not set MUTE_ISLA_LKAS12 or SUPPRESS_DAW_LKAS12
    fp6 = gen_empty_fingerprint()
    fp6[2][0x53E] = 6
    CP6 = CarInterface.get_params(CAR.HYUNDAI_ELANTRA_HEV_2024, fp6, [], False, False, False)
    CP_SP6 = CarInterface._get_params_sp(CP6, structs.CarParamsSP(), CAR.HYUNDAI_ELANTRA_HEV_2024, fp6, [], False, False, False)
    self.assertTrue(CP_SP6.flags & HyundaiFlagsSP.HAS_LKAS12)
    self.assertFalse(CP_SP6.flags & HyundaiFlagsSP.MUTE_ISLA_LKAS12)
    self.assertFalse(CP_SP6.flags & HyundaiFlagsSP.SUPPRESS_DAW_LKAS12)

  def test_create_lkas12_passthrough_and_suppression(self):
    # Real captured Elantra HEV 2024 DAW "Take a break" warning frame (b5 = 0x16, speed limit = 100 km/h)
    _, lkas12_warn = self._parse_cam("fc83002466005704", "0300646408166410")
    self.assertEqual(lkas12_warn["CF_LkasDawStatus"], 6)
    self.assertEqual(lkas12_warn["CF_Lkas_DawWarn"], 1)

    # Manual driving pass-through preserves all 8 bytes verbatim
    addr, dat, bus = hyundaican.create_lkas12(self.packer, lkas12_warn, mute_isla=False, suppress_daw=False)
    self.assertEqual((addr, bus), (0x53E, 0))
    self.assertEqual(dat.hex(), "0300646408166410")

    # Suppressing DAW + muting ISLA clamps DAW to 5/5 (b5 = 0x05), mutes ISLA chime, and preserves speed limit (100)
    _, dat_suppressed, _ = hyundaican.create_lkas12(self.packer, lkas12_warn, mute_isla=True, suppress_daw=True)
    self.pt_parser.update([(2_000_000, [(0x53E, dat_suppressed, 0)])])
    out = self.pt_parser.vl["LKAS12"]
    self.assertEqual(dat_suppressed[5], 0x05)
    self.assertEqual(out["CF_Lkas_DawSysState"], 3)
    self.assertEqual(out["CF_LkasDawStatus"], 5)
    self.assertEqual(out["CF_Lkas_DawWarn"], 0)
    self.assertEqual(out["CF_Lkas_TsrSpeed_Display_Clu"], 100)
    self.assertEqual(out["CF_LkasTsrSpeed_Display_Navi"], 100)
    self.assertEqual(out["CF_Lkas_TsrSlifOpt"], 2)
    self.assertEqual(out["ISLA_SpdWrn"], 1)

    # Overspeed chime (ISLA_SpdWrn = 3) + dropped DAW bar (b5 = 0x02) -> silent visual flash (1) + 5/5 DAW bars
    _, lkas12_drop = self._parse_cam("fc0300246600f704", "0300646408026430")
    _, dat_drop, _ = hyundaican.create_lkas12(self.packer, lkas12_drop, mute_isla=True, suppress_daw=True)
    self.pt_parser.update([(3_000_000, [(0x53E, dat_drop, 0)])])
    out_drop = self.pt_parser.vl["LKAS12"]
    self.assertEqual(dat_drop[5], 0x05)
    self.assertEqual(out_drop["CF_LkasDawStatus"], 5)
    self.assertEqual(out_drop["ISLA_SpdWrn"], 1)

  def test_create_lkas11_clears_daw_fcw_bas_req_but_preserves_fcw(self):
    fp = gen_empty_fingerprint()
    fp[2][0x53E] = 8
    CP = CarInterface.get_params(CAR.HYUNDAI_ELANTRA_HEV_2024, fp, [], False, False, False)

    lkas11_warn, _ = self._parse_cam("fc83002466005704", "0300646408166410")
    self.assertEqual(lkas11_warn["CF_Lkas_FcwBasReq"], 1)
    self.assertEqual(lkas11_warn["CF_Lkas_FcwCollisionWarning"], 0)

    # With suppress_daw=True and no FCW, CF_Lkas_FcwBasReq is cleared and CRC8 remains valid
    _, dat_suppressed, _ = hyundaican.create_lkas11(
      self.packer, 6, CP, 0, False, False, lkas11_warn, False, 3, True, True, True, 0, 0, 2, suppress_daw=True
    )
    self.pt_parser.update([(4_000_000, [(0x340, dat_suppressed, 0)])])
    self.assertEqual(self.pt_parser.vl["LKAS11"]["CF_Lkas_FcwBasReq"], 0)
    self.assertEqual(self.pt_parser.message_states[0x340].counter_fail, 0)

    # If camera asserts CF_Lkas_FcwCollisionWarning != 0, CF_Lkas_FcwBasReq is preserved
    lkas11_fcw = dict(lkas11_warn, CF_Lkas_FcwCollisionWarning=2)
    _, dat_fcw, _ = hyundaican.create_lkas11(
      self.packer, 7, CP, 0, False, False, lkas11_fcw, False, 3, True, True, True, 0, 0, 2, suppress_daw=True
    )
    self.pt_parser.update([(5_000_000, [(0x340, dat_fcw, 0)])])
    self.assertEqual(self.pt_parser.vl["LKAS11"]["CF_Lkas_FcwBasReq"], 1)

  def test_carcontroller_latches_in_progress_daw_warning_across_disengage(self):
    fp = gen_empty_fingerprint()
    fp[2][0x53E] = 8
    CP = CarInterface.get_params(CAR.HYUNDAI_ELANTRA_HEV_2024, fp, [], False, False, False)
    CP_SP = CarInterface._get_params_sp(CP, structs.CarParamsSP(), CAR.HYUNDAI_ELANTRA_HEV_2024, fp, [], False, False, False)
    dbc_names = {Bus.pt: "hyundai_can_generated"}

    CS = CarState(CP, CP_SP)
    CC_ctrl = CarController(dbc_names, CP, CP_SP)
    CS.lkas11, CS.lkas12 = self._parse_cam("fc83002466005704", "0300646408166410")
    CS.clu11 = {s: 0 for s in self.packer.dbc.name_to_msg["CLU11"].sigs}
    CS.out = structs.CarState()

    CC = structs.CarControl(enabled=False, latActive=False)
    CC_SP = structs.CarControlSP()
    CC_SP.mads.enabled = True

    # Frame 0: MADS active while camera emits DAW warning -> suppressed and latched
    CC_ctrl.frame = 0
    _, sends = CC_ctrl.update(CC.as_reader(), CC_SP, CS, 0)
    by_addr = {addr: dat for addr, dat, _ in sends}
    self.assertEqual(by_addr[0x53E][5], 0x05)
    self.assertEqual(by_addr[0x340][1] & 0x80, 0)
    self.assertTrue(CC_ctrl.daw_warn_latched)

    # Frame 10: Driver disengages MADS mid-warning -> remaining pulse stays suppressed
    CC_SP.mads.enabled = False
    CC_ctrl.frame = 10
    _, sends_disengaged = CC_ctrl.update(CC.as_reader(), CC_SP, CS, 100_000_000)
    by_addr_dis = {addr: dat for addr, dat, _ in sends_disengaged}
    self.assertEqual(by_addr_dis[0x53E][5], 0x05)
    self.assertEqual(by_addr_dis[0x340][1] & 0x80, 0)

    # Frame 20: Camera finishes 4s warning pulse -> latch clears and manual DAW level (1 bar) passes through
    CS.lkas11, CS.lkas12 = self._parse_cam("fc0300246600f704", "0300646408016410")
    CC_ctrl.frame = 20
    _, sends_cleared = CC_ctrl.update(CC.as_reader(), CC_SP, CS, 200_000_000)
    by_addr_clr = {addr: dat for addr, dat, _ in sends_cleared}
    self.assertFalse(CC_ctrl.daw_warn_latched)
    self.assertEqual(by_addr_clr[0x53E][5], 0x01)


if __name__ == "__main__":
  unittest.main()

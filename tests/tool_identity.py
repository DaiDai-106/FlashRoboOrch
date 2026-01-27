from robots_orchestra.driver.marvin.kinematics import Marvin_Kine
import glob

kine = Marvin_Kine()
ini_result = kine.load_config(config_path="/home/daidai/FlashRoboOrch/src/robots_orchestra/driver/config/ccs_m6.MvKDCfg")
print(f"ini_results:{ini_result}")
initial_kine_tag = kine.initial_kine(
    robot_serial=0,
    robot_type=ini_result["TYPE"][0],
    dh=ini_result["DH"][0],
    pnva=ini_result["PNVA"][0],
    j67=ini_result["BD"][0],
)


kine.identify_tool_dyn( 1, "/home/daidai/FlashRoboOrch/tests/data/")
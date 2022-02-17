import mc_rbdyn

json_path = "/path/to/file.json"
rm_json = mc_rbdyn.get_robot_module("json", json_path)

yaml_path = "/path/to/file.yaml"
rm_yaml = mc_rbdyn.get_robot_module("json", yaml_path)

import mc_rtc

import mc_rbdyn
import eigen
import sva

import tempfile

from nose.tools import *

def makeConfigFile(data):
  f = tempfile.NamedTemporaryFile(delete = False)
  f.write(data)
  f.close()
  return f.name

def sampleConfig(fromDisk):
  data ="""{
  "int": 42,
  "sint": -42,
  "double": 42.5,
  "doubleOrInt": 42.0,
  "string": "sometext",
  "intV": [0, 1, 2, 3, 4, 5],
  "stringV": ["a", "b", "c", "foo", "bar"],
  "doubleA3": [1.1, 2.2, 3.3],
  "v3d": [1.0, 2.3, -100],
  "v6d": [1.0, -1.5, 2.0, -2.5, 3.0, -3.5],
  "vXd": [0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10],
  "quat": [0.71, 0, 0.71, 0],
  "emptyArray": [],
  "boolTrue": true,
  "boolFalse": false,
  "bool1": 1,
  "bool0": 0,
  "doubleDoublePair": [42.5, -42.5],
  "doubleStringPair": [42.5, "sometext"],
  "doubleDoublePairV": [[0.0, 1.1],[2.2, 3.3], [4.4, 5.5]],
  "tuple" : [true, 42.42, "sometext", [1.2, 3.4, 5.6]],
  "dict":
  {
    "boolTrue": true,
    "bool0": 0,
    "int" : 42,
    "double": 42.5,
    "string": "sometext",
    "intV": [0, 1, 2, 3, 4, 5],
    "stringV": ["a", "b", "c", "foo", "bar"],
    "v3d": [1.0, 2.3, -100],
    "v6d": [1.0, -1.5, 2.0, -2.5, 3.0, -3.5],
    "quat": [0.71, 0, 0.71, 0],
    "doubleDoublePair": [42.5, -42.5],
    "doubleStringPair": [42.5, "sometext"]
  },
  "mapStr":
  {
    "str1": "sometext1",
    "str2": "sometext2",
    "str3": "sometext3",
    "str4": "sometext4"
  },
  "mapDouble":
  {
    "str1" : 1.1,
    "str2" : 2.2,
    "str3" : 3.3,
    "str4" : 4.4
  },
  "mapDoubleV":
  {
    "str1": [1.0, 2.3, -100],
    "str2": [1.0, -1.5, 2.0, -2.5, 3.0, -3.5],
    "str3": [0.71, 0, 0.71, 0]
  }
}"""
  if fromDisk:
    return makeConfigFile(data)
  else:
    return data

def sampleConfig2(fromDisk):
  data = """{
    "stringV": ["a2", "b2", "c2"],
    "int": 12
  }"""
  if fromDisk:
    return makeConfigFile(data)
  else:
    return data


@nottest
def test_configuration_reading(config, fromDisk2):
  @raises(RuntimeError)
  def test_throw():
    config("NONE")
  test_throw()

  assert(config("int", int) == 42)
  assert(config("int", 0) == 42)
  assert(config("NONE", 100) == 100)
  assert(config("dict")("int", int) == 42)

  assert(config("double", float) == 42.5)
  assert(config("double", 0.) == 42.5)
  assert(config("NONE", 42.42) == 42.42)
  assert(config("dict")("double", float) == 42.5)

  assert(config("string", str) == "sometext")
  assert(config("string", unicode) == "sometext")
  assert(config("string", "") == "sometext")
  assert(config("string", u"") == "sometext")
  assert(config("NONE", "another") == "another")
  assert(config("dict")("string", "") == "sometext")

  ref = eigen.Vector3d(1.0, 2.3, -100)
  zero = eigen.Vector3d.Zero()
  assert(config("v3d", eigen.Vector3d) == ref)
  assert(config("v3d", zero) == ref)
  assert(config("v6d", zero) == zero)
  @raises(RuntimeError)
  def test_v6d_to_v3d_throw():
    config("v6d", eigen.Vector3d)
  test_v6d_to_v3d_throw()
  assert(config("dict")("v3d", eigen.Vector3d) == ref)
  assert(config("dict")("v3d", zero) == ref)

  ref = eigen.Vector6d(1.0, -1.5, 2.0, -2.5, 3.0, -3.5)
  zero = eigen.Vector6d.Zero()
  assert(config("v6d", eigen.Vector6d) == ref)
  assert(config("v6d", zero) == ref)
  assert(config("v3d", zero) == zero)
  @raises(RuntimeError)
  def test_v3d_to_v6d_throw():
    config("v3d", eigen.Vector6d)
  test_v3d_to_v6d_throw()
  assert(config("dict")("v6d", eigen.Vector6d) == ref)
  assert(config("dict")("v6d", zero) == ref)

  ref3 = eigen.VectorXd(1, 2.3, -100)
  ref6 = eigen.VectorXd(ref)
  ref = eigen.VectorXd(0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10)
  assert(config("v6d", eigen.VectorXd) == ref6)
  assert(config("v3d", eigen.VectorXd) == ref3)
  assert(config("vXd", eigen.VectorXd) == ref)
  @raises(RuntimeError)
  def test_int_tovxd_throw():
    config("int", eigen.VectorXd)
  test_int_tovxd_throw()
  assert(config("dict")("v6d", eigen.VectorXd) == ref6)
  assert(config("dict")("v3d", eigen.VectorXd) == ref3)
  assert(config("emptyArray", eigen.VectorXd.Zero(100)) == eigen.VectorXd.Zero(0))

  ref = eigen.Quaterniond(0.71, 0, 0.71, 0)
  ref.normalize()
  assert(config("quat", eigen.Quaterniond).isApprox(ref))
  assert(config("dict")("quat", eigen.Quaterniond).isApprox(ref))

  assert(config("boolTrue", bool))
  assert(config("boolTrue", False))
  assert(not config("boolFalse", bool))
  assert(not config("boolFalse", True))
  assert(config("bool1", bool))
  assert(not config("bool0", bool))
  assert(config("dict")("boolTrue", bool))
  assert(not config("dict")("bool0", bool))

  ref = [0, 1, 2, 3, 4, 5]
  assert([c.to(int) for c in config("intV")] == ref)
  assert([c.to(int) for c in config("dict")("intV")] == ref)
  assert([c.to(float) for c in config("intV")] == ref)
  assert([c.to(float) for c in config("dict")("intV")] == ref)
  assert(config("intV", [int]) == ref)
  assert(config("intV", [0]) == ref)

  ref = ["a", "b", "c", "foo", "bar"]
  assert([c.to(str) for c in config("stringV")] == ref)
  assert([c.to(str) for c in config("dict")("stringV")] == ref)

  ref = [1.1, 2.2, 3.3]
  assert([c.to(float) for c in config("doubleA3")] == ref)

  ref = [42.5, -42.5]
  assert([c.to(float) for c in config("doubleDoublePair")] == ref)

  ref = [42.5, "sometext"]
  c = config("doubleStringPair")
  assert([c[0].to(float), c[1].to(str)] == ref)

  if fromDisk2:
    config.load(sampleConfig2(True))
  else:
    config.loadData(sampleConfig2(False))

  assert(config("int", int) == 12)
  assert(config("sint", int) == -42)

  ref = ["a2", "b2", "c2"]
  assert([c.to(str) for c in config("stringV")] == ref)

def test_configuration_reading_disk_disk():
  config = mc_rtc.Configuration(sampleConfig(True))
  test_configuration_reading(config, True)

def test_configuration_reading_disk_data():
  config = mc_rtc.Configuration(sampleConfig(True))
  test_configuration_reading(config, False)

def test_configuration_reading_data_disk():
  config = mc_rtc.Configuration.fromData(sampleConfig(False))
  test_configuration_reading(config, True)

def test_configuration_reading_data_data():
  config = mc_rtc.Configuration.fromData(sampleConfig(False))
  test_configuration_reading(config, False)

def test_configuration_writing():
  tmpF = tempfile.NamedTemporaryFile(delete = False).name
  config_ref = mc_rtc.Configuration()

  ref_bool = False;
  config_ref.add("bool", ref_bool)
  ref_uint = 42
  config_ref.add("uint", ref_uint)
  ref_int = -42
  config_ref.add("int", ref_int)
  ref_double = 42.5
  config_ref.add("double", ref_double)
  ref_string = "sometext"
  config_ref.add("string", ref_string)
  ref_v3d = eigen.Vector3d(1.2, 3.4, 5.6)
  config_ref.add("v3d", ref_v3d);
  ref_v6d = eigen.Vector6d(0.1, 1.2, 2.3, 3.4, 4.5, 5.6)
  config_ref.add("v6d", ref_v6d);
  ref_vxd = eigen.VectorXd(0.1, 3.2, 4.2, 4.5, 5.4)
  config_ref.add("vxd", ref_vxd);
  ref_quat = eigen.Quaterniond(0.71, 0, 0.71, 0)
  ref_quat.normalize()
  config_ref.add("quat", ref_quat)
  ref_int_v = [0, 1, 2, 3, 4, 5]
  config_ref.add("int_v", ref_int_v);
  ref_double_v = [0.1, 1.0, 0.2, 2.0, 0.3]
  config_ref.add("double_v", ref_double_v)
  ref_double_v_v = [ref_double_v, ref_double_v, [0], [], [5.0, 4.0, 3.5]]
  config_ref.add("double_v_v", ref_double_v_v)
  ref_v3d_v = [eigen.Vector3d.Random() for i in range(10)]
  config_ref.add("v3d_v", ref_v3d_v)
  config_ref.add("dict");
  config_ref("dict").add("int", ref_int);
  config_ref.add("dict2").add("double_v", ref_double_v);

  config_ref.save(tmpF)

  config_test = mc_rtc.Configuration(tmpF)
  assert(config_test("bool", bool) == ref_bool)
  assert(config_test("uint", int) == ref_uint)
  assert(config_test("int", int) == ref_int)
  assert(config_test("double", float) == ref_double)
  assert(config_test("string", str) == ref_string)
  assert(config_test("v3d", eigen.Vector3d) == ref_v3d)
  assert(config_test("v6d", eigen.Vector6d) == ref_v6d)
  assert(config_test("vxd", eigen.VectorXd) == ref_vxd)
  assert(config_test("quat", eigen.Quaterniond).isApprox(ref_quat))
  assert(config_test("int_v", [int]) == ref_int_v)
  assert(config_test("double_v", [float]) == ref_double_v)
  assert(config_test("double_v_v", [[0.]]) == ref_double_v_v)
  assert(all([(v - ref_v).norm() < 1e-9 for v, ref_v in zip(config_test("v3d_v", [eigen.Vector3d]), ref_v3d_v)]))
  assert(config_test("dict")("int", int) == ref_int)
  assert(config_test("dict2")("double_v", [float]) == ref_double_v)

  config_test("dict2")("double_v").save(tmpF)
  assert(mc_rtc.Configuration(tmpF).to([float]) == ref_double_v)

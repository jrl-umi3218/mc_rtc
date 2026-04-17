/*
 * Copyright 2015-2023 CNRS-UM LIRMM, CNRS-AIST JRL
 */

#include "mpack.h"

#include <mc_rtc/Configuration.h>

namespace mc_rtc::internal
{

namespace
{

void fromMessagePackArray(mc_rtc::Configuration config, mpack_node_t node);

void fromMessagePackMap(mc_rtc::Configuration config, mpack_node_t node);

inline std::string toString(mpack_node_t node)
{
  return {mpack_node_str(node), mpack_node_strlen(node)};
}

/** Add data into a map */
inline void fromMessagePack(mc_rtc::Configuration config, const std::string & key, mpack_node_t node)
{
  switch(mpack_node_type(node))
  {
    case mpack_type_missing:
      break;
    case mpack_type_nil:
      config.add_null(key);
      break;
    case mpack_type_bool:
      config.add(key, mpack_node_bool(node));
      break;
    case mpack_type_int:
      config.add(key, mpack_node_i64(node));
      break;
    case mpack_type_uint:
      config.add(key, mpack_node_u64(node));
      break;
    case mpack_type_float:
      config.add(key, mpack_node_float(node));
      break;
    case mpack_type_double:
      config.add(key, mpack_node_double(node));
      break;
    case mpack_type_str:
      config.add(key, toString(node));
      break;
    case mpack_type_array:
      fromMessagePackArray(config.array(key, mpack_node_array_length(node)), node);
      break;
    case mpack_type_map:
      fromMessagePackMap(config.add(key), node);
      break;
    default:
      log::error_and_throw("Unsupported type in MessagePack");
  }
}

/** Add data into an array */
inline void fromMessagePack(mc_rtc::Configuration config, mpack_node_t node)
{
  switch(mpack_node_type(node))
  {
    case mpack_type_missing:
      break;
    case mpack_type_nil:
      config.push_null();
      break;
    case mpack_type_bool:
      config.push(mpack_node_bool(node));
      break;
    case mpack_type_int:
      config.push(mpack_node_i64(node));
      break;
    case mpack_type_uint:
      config.push(mpack_node_u64(node));
      break;
    case mpack_type_float:
      config.push(mpack_node_float(node));
      break;
    case mpack_type_double:
      config.push(mpack_node_double(node));
      break;
    case mpack_type_str:
      config.push(toString(node));
      break;
    case mpack_type_array:
      fromMessagePackArray(config.array(mpack_node_array_length(node)), node);
      break;
    case mpack_type_map:
      fromMessagePackMap(config.object(), node);
      break;
    default:
      log::error_and_throw("Unsupported type in MessagePack");
  }
}

inline void fromMessagePackArray(mc_rtc::Configuration config, mpack_node_t node)
{
  for(size_t i = 0; i < mpack_node_array_length(node); ++i)
  {
    fromMessagePack(config, mpack_node_array_at(node, i));
  }
}

inline void fromMessagePackMap(mc_rtc::Configuration config, mpack_node_t node)
{
  for(size_t i = 0; i < mpack_node_map_count(node); ++i)
  {
    fromMessagePack(config, toString(mpack_node_map_key_at(node, i)), mpack_node_map_value_at(node, i));
  }
}

} // namespace

inline mc_rtc::Configuration fromMessagePack(mpack_node_t root)
{
  mc_rtc::Configuration config;
  if(mpack_node_type(root) == mpack_type_map)
  {
    fromMessagePackMap(config, root);
  }
  else if(mpack_node_type(root) == mpack_type_array)
  {
    config = mc_rtc::Configuration::rootArray();
    fromMessagePackArray(config, root);
  }
  else
  {
    fromMessagePack(config, "data", root);
    config = config("data");
  }
  return config;
}

inline void fromMessagePack(mc_rtc::Configuration & config, const char * data, size_t size)
{
  mpack_tree_t tree;
  mpack_tree_init_data(&tree, data, size);
  mpack_tree_parse(&tree);
  if(mpack_tree_error(&tree) != mpack_ok)
  {
    log::error("Failed to parse MessagePack data");
    return;
  }
  auto root = mpack_tree_root(&tree);
  config = fromMessagePack(root);
  mpack_tree_destroy(&tree);
}

} // namespace mc_rtc::internal

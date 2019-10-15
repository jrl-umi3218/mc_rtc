#include <mc_rtc/Configuration.h>
#include <mc_rtc/MessagePackBuilder.h>
#include <mc_rtc/logging.h>

#include "mpack.h"

#if not EIGEN_VERSION_AT_LEAST(3, 2, 90)
namespace Eigen
{
using Index = Eigen::DenseIndex;
}
#endif

namespace mc_rtc
{

struct MessagePackBuilderImpl : mpack_writer_t
{
};

/** Inspired by mpack.c @ version 1.0 */
static void mpack_std_vector_writer_flush(mpack_writer_t * writer, const char * data, size_t count)
{
  auto & buffer = *static_cast<std::vector<char> *>(writer->context);
  // This is an intrusive flush function which modifies the writer's buffer
  // in response to a flush instead of emptying it in order to add more
  // capacity for data. This removes the need to copy data from a fixed buffer
  // into a growable one, improving performance.
  //
  // There are three ways flush can be called:
  //   - flushing the buffer during writing (used is zero, count is all data, data is buffer)
  //   - flushing extra data during writing (used is all flushed data, count is extra data, data is not buffer)
  //   - flushing during teardown (used and count are both all flushed data, data is buffer)
  //
  // In the first two cases, we grow the buffer by at least double, enough
  // to ensure that new data will fit. We ignore the teardown flush.

  if(data == writer->buffer)
  {

    // teardown, do nothing
    if(mpack_writer_buffer_used(writer) == count) return;

    // otherwise leave the data in the buffer and just grow
    writer->current = writer->buffer + count;
    count = 0;
  }

  size_t used = mpack_writer_buffer_used(writer);
  size_t size = mpack_writer_buffer_size(writer);

  mpack_log("flush size %i used %i data %p buffer %p\n", (int)count, (int)used, data, writer->buffer);

  mpack_assert(data == writer->buffer || used + count > size,
               "extra flush for %i but there is %i space left in the buffer! (%i/%i)", (int)count,
               (int)mpack_writer_buffer_left(writer), (int)used, (int)size);

  // grow to fit the data
  // TODO: this really needs to correctly test for overflow
  size_t new_size = size * 2;
  while(new_size < used + count) new_size *= 2;

  mpack_log("flush growing buffer size from %i to %i\n", (int)size, (int)new_size);

  // grow the buffer
  buffer.resize(new_size);
  char * new_buffer = buffer.data();
  if(new_buffer == NULL)
  {
    mpack_writer_flag_error(writer, mpack_error_memory);
    return;
  }
  writer->current = new_buffer + used;
  writer->buffer = new_buffer;
  writer->end = writer->buffer + new_size;

  // append the extra data
  if(count > 0)
  {
    mpack_memcpy(writer->current, data, count);
    writer->current += count;
  }

  mpack_log("new buffer %p, used %i\n", new_buffer, (int)mpack_writer_buffer_used(writer));
}

MessagePackBuilder::MessagePackBuilder(std::vector<char> & buffer) : impl_(new MessagePackBuilderImpl())
{
  if(buffer.size() == 0)
  {
    buffer.resize(MPACK_BUFFER_SIZE);
  }
  mpack_writer_init(impl_.get(), buffer.data(), buffer.size());
  mpack_writer_set_context(impl_.get(), &buffer);
  mpack_writer_set_flush(impl_.get(), mpack_std_vector_writer_flush);
}

MessagePackBuilder::~MessagePackBuilder() {}

void MessagePackBuilder::write()
{
  mpack_write_nil(impl_.get());
}

void MessagePackBuilder::write(bool b)
{
  mpack_write_bool(impl_.get(), b);
}
void MessagePackBuilder::write(int8_t i)
{
  mpack_write_i8(impl_.get(), i);
}
void MessagePackBuilder::write(int16_t i)
{
  mpack_write_i16(impl_.get(), i);
}
void MessagePackBuilder::write(int32_t i)
{
  mpack_write_i32(impl_.get(), i);
}
void MessagePackBuilder::write(int64_t i)
{
  mpack_write_i64(impl_.get(), i);
}
void MessagePackBuilder::write(uint8_t i)
{
  mpack_write_u8(impl_.get(), i);
}
void MessagePackBuilder::write(uint16_t i)
{
  mpack_write_u16(impl_.get(), i);
}
void MessagePackBuilder::write(uint32_t i)
{
  mpack_write_u32(impl_.get(), i);
}
void MessagePackBuilder::write(uint64_t i)
{
  mpack_write_u64(impl_.get(), i);
}
void MessagePackBuilder::write(float f)
{
  mpack_write_float(impl_.get(), f);
}
void MessagePackBuilder::write(double d)
{
  mpack_write_double(impl_.get(), d);
}
void MessagePackBuilder::write(const std::string & s)
{
  mpack_write_str(impl_.get(), s.c_str(), static_cast<uint32_t>(s.size()));
}
void MessagePackBuilder::write(const char * s)
{
  mpack_write_cstr(impl_.get(), s);
}

void MessagePackBuilder::write(const char * s, size_t len)
{
  mpack_write_str(impl_.get(), s, static_cast<uint32_t>(len));
}

namespace
{

template<typename T>
inline void write_vector(mpack_writer_t * writer, const T & v)
{
  for(Eigen::Index i = 0; i < v.size(); ++i)
  {
    mpack_write_double(writer, v(i));
  }
}

template<typename T>
inline void write_matrix(mpack_writer_t * writer, const T & m)
{
  for(Eigen::Index i = 0; i < m.rows(); ++i)
  {
    for(Eigen::Index j = 0; j < m.cols(); ++j)
    {
      mpack_write_double(writer, m(i, j));
    }
  }
}

} // namespace

void MessagePackBuilder::write(const Eigen::Vector2d & v)
{
  start_array(2);
  write_vector(impl_.get(), v);
  finish_array();
}

void MessagePackBuilder::write(const Eigen::Vector3d & v)
{
  start_array(3);
  write_vector(impl_.get(), v);
  finish_array();
}

void MessagePackBuilder::write(const Eigen::Vector6d & v)
{
  start_array(6);
  write_vector(impl_.get(), v);
  finish_array();
}

void MessagePackBuilder::write(const Eigen::VectorXd & v)
{
  start_array(static_cast<size_t>(v.size()));
  write_vector(impl_.get(), v);
  finish_array();
}

void MessagePackBuilder::write(const Eigen::Quaterniond & q)
{
  start_array(4);
  write(q.w());
  write(q.x());
  write(q.y());
  write(q.z());
  finish_array();
}

void MessagePackBuilder::write(const Eigen::Matrix3d & m)
{
  start_array(9);
  write_matrix(impl_.get(), m);
  finish_array();
}

void MessagePackBuilder::write(const sva::PTransformd & pt)
{
  start_array(12);
  write_matrix(impl_.get(), pt.rotation());
  write_vector(impl_.get(), pt.translation());
  finish_array();
}

void MessagePackBuilder::write(const sva::ForceVecd & fv)
{
  write(fv.vector());
}

void MessagePackBuilder::write(const sva::MotionVecd & mv)
{
  write(mv.vector());
}

void MessagePackBuilder::write(const mc_rtc::Configuration & config)
{
  config.toMessagePack(*this);
}

void MessagePackBuilder::start_array(size_t s)
{
  mpack_start_array(impl_.get(), static_cast<uint32_t>(s));
}

void MessagePackBuilder::finish_array()
{
  mpack_finish_array(impl_.get());
}

void MessagePackBuilder::start_map(size_t s)
{
  mpack_start_map(impl_.get(), static_cast<uint32_t>(s));
}

void MessagePackBuilder::finish_map()
{
  mpack_finish_map(impl_.get());
}

void MessagePackBuilder::write_object(const char * data, size_t s)
{
  mpack_write_object_bytes(impl_.get(), data, s);
}

size_t MessagePackBuilder::finish()
{
  if(mpack_writer_destroy(impl_.get()) != mpack_ok)
  {
    LOG_ERROR("Failed to convert to MessagePack")
    return 0;
  }
  return mpack_writer_buffer_used(impl_.get());
}

} // namespace mc_rtc

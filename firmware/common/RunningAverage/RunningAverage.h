#pragma once

#include <array>

template <typename T, size_t S> class RunningAverage
{
public:
  RunningAverage()
  {
    m_array.fill(0);
  }

  void add(T _value)
  {
    m_sum -= m_array[m_index];
    m_array[m_index] = _value;
    m_sum += m_array[m_index];
    m_index++;
    if(m_index == S) m_index = 0;
    if(m_count < S) m_count++;
  }

  float getAverage() const { return m_sum / static_cast<float>(m_count); }

  size_t getCount() const { return m_count; }

private:
  size_t m_index = 0;
  size_t m_count = 0;
  T m_sum = 0;
  std::array<T, S> m_array;
};

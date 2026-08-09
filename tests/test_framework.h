#pragma once

#include <cmath>
#include <cstdlib>
#include <functional>
#include <iostream>
#include <string>
#include <utility>
#include <vector>

#define FF_SKIP_EXIT_CODE 77

namespace fftest
{

struct FRequireFailed
{
};
struct FSkipped
{
  std::string reason;
};

struct FTestCase
{
  std::string           name;
  std::function<void()> fn;
};

inline std::vector<FTestCase>& Registry()
{
  static std::vector<FTestCase> registry;
  return registry;
}

inline int& Failures()
{
  static int failures = 0;
  return failures;
}

struct FRegistrar
{
  FRegistrar(std::string name, std::function<void()> fn)
  {
    Registry().push_back({std::move(name), std::move(fn)});
  }
};

inline bool Report(const bool bOk, const char* What, const char* File, const int Line)
{
  if (!bOk) {
    ++Failures();
    std::cerr << "    FAIL " << File << ":" << Line << "  " << What << std::endl;
  }
  return bOk;
}

[[noreturn]] inline void Skip(const std::string& Reason)
{
  throw FSkipped{Reason};
}

inline int RunAll()
{
  int ran     = 0;
  int skipped = 0;

  for (const FTestCase& Test : Registry()) {
    const int Before = Failures();
    std::cout << "[ RUN  ] " << Test.name << std::endl;

    try {
      Test.fn();
      ++ran;
      std::cout << (Failures() == Before ? "[  OK  ] " : "[ FAIL ] ") << Test.name << std::endl;
    }
    catch (const FSkipped& S) {
      ++skipped;
      std::cout << "[ SKIP ] " << Test.name << " (" << S.reason << ")" << std::endl;
    }
    catch (const FRequireFailed&) {
      ++ran;
      std::cout << "[ FAIL ] " << Test.name << " (requirement failed)" << std::endl;
    }
    catch (const std::exception& E) {
      ++Failures();
      ++ran;
      std::cout << "[ FAIL ] " << Test.name << " (exception: " << E.what() << ")" << std::endl;
    }
  }

  std::cout << "----\n" << ran << " ran, " << skipped << " skipped, " << Failures() << " failure(s)" << std::endl;

  if (Failures() > 0) {
    return EXIT_FAILURE;
  }
  if (ran == 0 && skipped > 0) {
    return FF_SKIP_EXIT_CODE;
  }
  return EXIT_SUCCESS;
}

}  // namespace fftest

#define FF_TEST(Name)                                                    \
  static void        Name();                                             \
  static ::fftest::FRegistrar Name##_registrar(#Name, &Name);            \
  static void        Name()

#define FF_CHECK(Cond) ::fftest::Report(static_cast<bool>(Cond), #Cond, __FILE__, __LINE__)

#define FF_CHECK_NEAR(A, B, Tol) ::fftest::Report(std::fabs((A) - (B)) <= (Tol), #A " ~= " #B, __FILE__, __LINE__)

#define FF_REQUIRE(Cond)                                                 \
  do {                                                                   \
    if (!::fftest::Report(static_cast<bool>(Cond), #Cond, __FILE__, __LINE__)) { \
      throw ::fftest::FRequireFailed{};                                  \
    }                                                                    \
  } while (0)

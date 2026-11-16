#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <vector>

#include <ftxui/component/component.hpp>

// A form cell is bound to one value inside an action's parameter struct. The
// text buffer is what the Input edits; Push() parses it into the value right
// before the call is sent, Pull() refreshes the text after a "Load current".
struct Field
{
  enum Kind
  {
    Double,
    Int,
    UInt,
    Bool,
    Enum,     // int target, choice between labelled values
    StrEnum,  // std::string target, choice between strings
    String,
  };

  Kind        kind = String;
  std::string label;  // short sub-label shown before the cell ("x", "y", ...)
  void*       ptr  = nullptr;
  int         width = 10;  // cell width for numeric inputs, 0 = flex

  std::vector<std::string> opt_labels;
  std::vector<int>         opt_values;

  // UI-side state
  std::string text;
  bool        bval   = false;
  int         choice = 0;

  void        Pull();
  bool        Push(std::string& error);
  std::string Format() const;  // current bound value, for read-outs

  static Field D(double* v, std::string label = "", int width = 10);
  static Field I(int* v, std::string label = "", int width = 8);
  static Field U(unsigned* v, std::string label = "", int width = 10);
  static Field B(bool* v, std::string label = "");
  static Field E(int* v, std::vector<std::pair<std::string, int>> options, std::string label = "");
  static Field SE(std::string* v, std::vector<std::string> options, std::string label = "");
  static Field S(std::string* v, std::string label = "", int width = 0);
};

struct FieldRow
{
  std::string        label;
  std::vector<Field> cells;
  bool               separator = false;  // section heading, no cells
  std::string        hint;               // shown dim after the cells

  static FieldRow Section(std::string title) {
    FieldRow row;
    row.label     = std::move(title);
    row.separator = true;
    return row;
  }
};

FieldRow Row(std::string label, std::vector<Field> cells, std::string hint = "");

// common cell groups
std::vector<Field> Vec3(double* x, double* y, double* z, const char* a = "x", const char* b = "y", const char* c = "z");

void PullAll(std::vector<FieldRow>& rows);
bool PushAll(std::vector<FieldRow>& rows, std::string& error);
std::vector<std::string> DescribeRows(const std::vector<FieldRow>& rows);

struct FormCallbacks
{
  std::function<void()> execute;
  std::function<void()> load;  // empty: no "Load current" button
  std::function<void()> back;  // Esc
};

// The form component for one action; rows must outlive it.
ftxui::Component BuildForm(std::vector<FieldRow>& rows, const FormCallbacks& callbacks);

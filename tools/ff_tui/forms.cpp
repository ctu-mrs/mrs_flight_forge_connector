#include "forms.h"

#include <cstdio>
#include <stdexcept>

#include <ftxui/component/component_base.hpp>
#include <ftxui/component/event.hpp>
#include <ftxui/dom/elements.hpp>

using namespace ftxui;

namespace
{

std::string FormatDouble(double v) {
  char buf[64];
  std::snprintf(buf, sizeof(buf), "%.6g", v);
  return buf;
}

std::string Trim(const std::string& s) {
  const auto b = s.find_first_not_of(" \t");
  if (b == std::string::npos) {
    return "";
  }
  const auto e = s.find_last_not_of(" \t");
  return s.substr(b, e - b + 1);
}

// "◂ value ▸" selector; ←/→ cycle, Enter executes
class ChoiceCell : public ComponentBase {
public:
  ChoiceCell(Field* field, std::function<void()> on_enter) : field_(field), on_enter_(std::move(on_enter)) {
  }

  Element Render() override {
    const int   n     = static_cast<int>(field_->opt_labels.size());
    std::string value = n > 0 ? field_->opt_labels[field_->choice] : "-";
    auto        e     = hbox({text("◂ "), text(value), text(" ▸")});
    if (Focused()) {
      return e | inverted | focus;
    }
    return e | color(Color::Cyan);
  }

  bool OnEvent(Event event) override {
    if (!Focused()) {
      return false;
    }
    const int n = static_cast<int>(field_->opt_labels.size());
    if (n == 0) {
      return false;
    }
    if (event == Event::ArrowLeft || event == Event::Character('h')) {
      field_->choice = (field_->choice - 1 + n) % n;
      return true;
    }
    if (event == Event::ArrowRight || event == Event::Character('l')) {
      field_->choice = (field_->choice + 1) % n;
      return true;
    }
    if (event == Event::Return) {
      if (on_enter_) {
        on_enter_();
      }
      return true;
    }
    return false;
  }

  bool Focusable() const override {
    return true;
  }

private:
  Field*                field_;
  std::function<void()> on_enter_;
};

// [x] toggle; Space/Enter flip it, it never executes on its own
class BoolCell : public ComponentBase {
public:
  explicit BoolCell(Field* field) : field_(field) {
  }

  Element Render() override {
    auto e = text(field_->bval ? "[x] on " : "[ ] off");
    if (Focused()) {
      return e | inverted | focus;
    }
    return e | color(field_->bval ? Color::Green : Color::GrayLight);
  }

  bool OnEvent(Event event) override {
    if (!Focused()) {
      return false;
    }
    if (event == Event::Return || event == Event::Character(' ')) {
      field_->bval = !field_->bval;
      return true;
    }
    return false;
  }

  bool Focusable() const override {
    return true;
  }

private:
  Field* field_;
};

Component MakeCell(Field& field, const std::function<void()>& execute) {
  switch (field.kind) {
    case Field::Bool:
      return Make<BoolCell>(&field);
    case Field::Enum:
    case Field::StrEnum:
      return Make<ChoiceCell>(&field, execute);
    default:
      break;
  }

  InputOption option;
  option.multiline = false;
  option.content   = &field.text;
  option.on_enter  = execute;
  option.transform = [](InputState state) {
    auto e = state.element;
    if (state.is_placeholder) {
      e |= dim;
    }
    if (state.focused) {
      return e | color(Color::Black) | bgcolor(Color::Cyan);
    }
    return e | bgcolor(Color::GrayDark);
  };
  return Input(option);
}

}  // namespace

/* Field //{ */

Field Field::D(double* v, std::string label, int width) {
  Field f;
  f.kind  = Double;
  f.ptr   = v;
  f.label = std::move(label);
  f.width = width;
  return f;
}

Field Field::I(int* v, std::string label, int width) {
  Field f;
  f.kind  = Int;
  f.ptr   = v;
  f.label = std::move(label);
  f.width = width;
  return f;
}

Field Field::U(unsigned* v, std::string label, int width) {
  Field f;
  f.kind  = UInt;
  f.ptr   = v;
  f.label = std::move(label);
  f.width = width;
  return f;
}

Field Field::B(bool* v, std::string label) {
  Field f;
  f.kind  = Bool;
  f.ptr   = v;
  f.label = std::move(label);
  return f;
}

Field Field::E(int* v, std::vector<std::pair<std::string, int>> options, std::string label) {
  Field f;
  f.kind  = Enum;
  f.ptr   = v;
  f.label = std::move(label);
  for (auto& [name, value] : options) {
    f.opt_labels.push_back(name);
    f.opt_values.push_back(value);
  }
  return f;
}

Field Field::SE(std::string* v, std::vector<std::string> options, std::string label) {
  Field f;
  f.kind       = StrEnum;
  f.ptr        = v;
  f.label      = std::move(label);
  f.opt_labels = std::move(options);
  return f;
}

Field Field::S(std::string* v, std::string label, int width) {
  Field f;
  f.kind  = String;
  f.ptr   = v;
  f.label = std::move(label);
  f.width = width;
  return f;
}

std::string Field::Format() const {
  switch (kind) {
    case Double:
      return FormatDouble(*static_cast<double*>(ptr));
    case Int:
      return std::to_string(*static_cast<int*>(ptr));
    case UInt:
      return std::to_string(*static_cast<unsigned*>(ptr));
    case Bool:
      return *static_cast<bool*>(ptr) ? "true" : "false";
    case Enum: {
      const int v = *static_cast<int*>(ptr);
      for (size_t i = 0; i < opt_values.size(); ++i) {
        if (opt_values[i] == v) {
          return opt_labels[i];
        }
      }
      return std::to_string(v);
    }
    case StrEnum:
    case String:
      return *static_cast<std::string*>(ptr);
  }
  return "";
}

void Field::Pull() {
  switch (kind) {
    case Bool:
      bval = *static_cast<bool*>(ptr);
      break;
    case Enum: {
      const int v = *static_cast<int*>(ptr);
      choice      = 0;
      for (size_t i = 0; i < opt_values.size(); ++i) {
        if (opt_values[i] == v) {
          choice = static_cast<int>(i);
        }
      }
      break;
    }
    case StrEnum: {
      const auto& v = *static_cast<std::string*>(ptr);
      choice        = 0;
      for (size_t i = 0; i < opt_labels.size(); ++i) {
        if (opt_labels[i] == v) {
          choice = static_cast<int>(i);
        }
      }
      break;
    }
    default:
      text = Format();
      break;
  }
}

bool Field::Push(std::string& error) {
  const std::string t = Trim(text);
  try {
    size_t used = 0;
    switch (kind) {
      case Double: {
        const double v = std::stod(t, &used);
        if (used != t.size()) {
          throw std::invalid_argument("trailing");
        }
        *static_cast<double*>(ptr) = v;
        break;
      }
      case Int: {
        const int v = std::stoi(t, &used);
        if (used != t.size()) {
          throw std::invalid_argument("trailing");
        }
        *static_cast<int*>(ptr) = v;
        break;
      }
      case UInt: {
        const unsigned long v = std::stoul(t, &used);
        if (used != t.size()) {
          throw std::invalid_argument("trailing");
        }
        *static_cast<unsigned*>(ptr) = static_cast<unsigned>(v);
        break;
      }
      case Bool:
        *static_cast<bool*>(ptr) = bval;
        break;
      case Enum:
        if (!opt_values.empty()) {
          *static_cast<int*>(ptr) = opt_values[choice];
        }
        break;
      case StrEnum:
        if (!opt_labels.empty()) {
          *static_cast<std::string*>(ptr) = opt_labels[choice];
        }
        break;
      case String:
        *static_cast<std::string*>(ptr) = t;
        break;
    }
  }
  catch (const std::exception&) {
    error = "'" + t + "' is not a valid " + (kind == Double ? "number" : "integer") + (label.empty() ? "" : " for " + label);
    return false;
  }
  return true;
}

//}

FieldRow Row(std::string label, std::vector<Field> cells, std::string hint) {
  FieldRow row;
  row.label = std::move(label);
  row.cells = std::move(cells);
  row.hint  = std::move(hint);
  return row;
}

std::vector<Field> Vec3(double* x, double* y, double* z, const char* a, const char* b, const char* c) {
  return {Field::D(x, a), Field::D(y, b), Field::D(z, c)};
}

void PullAll(std::vector<FieldRow>& rows) {
  for (auto& row : rows) {
    for (auto& cell : row.cells) {
      cell.Pull();
    }
  }
}

bool PushAll(std::vector<FieldRow>& rows, std::string& error) {
  for (auto& row : rows) {
    for (auto& cell : row.cells) {
      if (!cell.Push(error)) {
        error = row.label + ": " + error;
        return false;
      }
    }
  }
  return true;
}

std::vector<std::string> DescribeRows(const std::vector<FieldRow>& rows) {
  std::vector<std::string> lines;
  for (const auto& row : rows) {
    if (row.separator) {
      lines.push_back("── " + row.label);
      continue;
    }
    std::string line = "  " + row.label + ":";
    for (const auto& cell : row.cells) {
      line += " ";
      if (!cell.label.empty()) {
        line += cell.label + "=";
      }
      line += cell.Format();
    }
    lines.push_back(line);
  }
  return lines;
}

Component BuildForm(std::vector<FieldRow>& rows, const FormCallbacks& callbacks) {
  auto container = Container::Vertical({});

  for (auto& row : rows) {
    if (row.separator) {
      const std::string title = row.label;
      container->Add(Renderer([title] { return hbox({text("── " + title + " ") | bold | color(Color::Yellow), separatorLight() | flex}); }));
      continue;
    }

    auto cells = Container::Horizontal({});
    std::vector<Component> components;
    for (auto& cell : row.cells) {
      auto c = MakeCell(cell, callbacks.execute);
      components.push_back(c);
      cells->Add(c);
    }

    FieldRow* row_ptr = &row;
    container->Add(Renderer(cells, [row_ptr, components] {
      Elements parts;
      parts.push_back(text(row_ptr->label) | size(WIDTH, EQUAL, 22) | color(Color::GrayLight));
      for (size_t i = 0; i < components.size(); ++i) {
        const auto& cell = row_ptr->cells[i];
        if (!cell.label.empty()) {
          parts.push_back(text(" " + cell.label + " ") | dim);
        } else if (i > 0) {
          parts.push_back(text(" "));
        }
        auto e = components[i]->Render();
        if ((cell.kind == Field::Double || cell.kind == Field::Int || cell.kind == Field::UInt || cell.kind == Field::String) && cell.width > 0) {
          e = e | size(WIDTH, EQUAL, cell.width);
        } else if (cell.kind == Field::String) {
          e = e | flex;
        }
        parts.push_back(e);
      }
      if (!row_ptr->hint.empty()) {
        parts.push_back(text("  " + row_ptr->hint) | dim);
      }
      return hbox(parts);
    }));
  }

  auto buttons      = Container::Horizontal({});
  auto button_style = ButtonOption::Ascii();
  buttons->Add(Button(" Execute ⏎ ", callbacks.execute, button_style));
  if (callbacks.load) {
    buttons->Add(Button(" Load current ", callbacks.load, button_style));
  }
  container->Add(Renderer(buttons, [buttons] { return vbox({text(""), buttons->Render()}); }));

  auto back = callbacks.back;
  return CatchEvent(container, [back](Event event) {
    if (event == Event::Escape && back) {
      back();
      return true;
    }
    return false;
  });
}

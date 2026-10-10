// Small JSON reader for track / robot / scenario files.
#pragma once
#include <string>
#include <utility>
#include <vector>

namespace json {

struct Value {
	enum Type { Null, Bool, Number, String, Array, Object } type = Null;
	bool b = false;
	double n = 0.0;
	std::string s;
	std::vector<Value> a;
	std::vector<std::pair<std::string, Value>> o;

	bool is_null() const { return type == Null; }
	bool is_num() const { return type == Number; }
	bool is_str() const { return type == String; }
	bool is_arr() const { return type == Array; }
	bool is_obj() const { return type == Object; }

	const Value* get(const std::string& key) const;   // nullptr if absent
	bool has(const std::string& key) const { return get(key) != nullptr; }
	double num(const std::string& key, double def) const;
	std::string str(const std::string& key, const std::string& def) const;
	bool boolean(const std::string& key, bool def) const;
	size_t size() const { return type == Array ? a.size() : (type == Object ? o.size() : 0); }
	const Value& operator[](size_t i) const { return a[i]; }
};

Value parse(const std::string& text, std::string* err);
bool parse_file(const std::string& path, Value* out, std::string* err);

}  // namespace json

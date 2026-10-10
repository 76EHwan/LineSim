#include "core/json.h"

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace json {

const Value* Value::get(const std::string& key) const {
	if (type != Object)
		return nullptr;
	for (const auto& kv : o)
		if (kv.first == key)
			return &kv.second;
	return nullptr;
}

double Value::num(const std::string& key, double def) const {
	const Value* v = get(key);
	return (v && v->type == Number) ? v->n : def;
}

std::string Value::str(const std::string& key, const std::string& def) const {
	const Value* v = get(key);
	return (v && v->type == String) ? v->s : def;
}

bool Value::boolean(const std::string& key, bool def) const {
	const Value* v = get(key);
	return (v && v->type == Bool) ? v->b : def;
}

namespace {

struct Parser {
	const std::string& t;
	size_t i = 0;
	std::string err;

	explicit Parser(const std::string& text) : t(text) {}

	void ws() {
		for (;;) {
			while (i < t.size() && (t[i] == ' ' || t[i] == '\t' || t[i] == '\n' || t[i] == '\r'))
				i++;
			// allow // line comments in config files
			if (i + 1 < t.size() && t[i] == '/' && t[i + 1] == '/') {
				while (i < t.size() && t[i] != '\n')
					i++;
				continue;
			}
			break;
		}
	}

	bool fail(const std::string& m) {
		if (err.empty()) {
			size_t line = 1;
			for (size_t k = 0; k < i && k < t.size(); k++)
				if (t[k] == '\n')
					line++;
			err = m + " (line " + std::to_string(line) + ")";
		}
		return false;
	}

	bool value(Value& v) {
		ws();
		if (i >= t.size())
			return fail("unexpected end");
		char c = t[i];
		if (c == '{')
			return object(v);
		if (c == '[')
			return array(v);
		if (c == '"') {
			v.type = Value::String;
			return string(v.s);
		}
		if (t.compare(i, 4, "true") == 0) {
			v.type = Value::Bool;
			v.b = true;
			i += 4;
			return true;
		}
		if (t.compare(i, 5, "false") == 0) {
			v.type = Value::Bool;
			v.b = false;
			i += 5;
			return true;
		}
		if (t.compare(i, 4, "null") == 0) {
			v.type = Value::Null;
			i += 4;
			return true;
		}
		const char* start = t.c_str() + i;
		char* end = nullptr;
		double d = std::strtod(start, &end);
		if (end == start)
			return fail("bad value");
		v.type = Value::Number;
		v.n = d;
		i += static_cast<size_t>(end - start);
		return true;
	}

	bool string(std::string& out) {
		i++;  // opening quote
		while (i < t.size() && t[i] != '"') {
			char c = t[i++];
			if (c == '\\' && i < t.size()) {
				char e = t[i++];
				switch (e) {
				case 'n': out += '\n'; break;
				case 't': out += '\t'; break;
				case 'r': out += '\r'; break;
				case 'u': i += 4; out += '?'; break;
				default: out += e; break;
				}
			} else {
				out += c;
			}
		}
		if (i >= t.size())
			return fail("unterminated string");
		i++;
		return true;
	}

	bool array(Value& v) {
		v.type = Value::Array;
		i++;
		ws();
		if (i < t.size() && t[i] == ']') {
			i++;
			return true;
		}
		for (;;) {
			Value e;
			if (!value(e))
				return false;
			v.a.push_back(std::move(e));
			ws();
			if (i < t.size() && t[i] == ',') {
				i++;
				ws();
				if (i < t.size() && t[i] == ']') {  // trailing comma
					i++;
					return true;
				}
				continue;
			}
			if (i < t.size() && t[i] == ']') {
				i++;
				return true;
			}
			return fail("expected , or ]");
		}
	}

	bool object(Value& v) {
		v.type = Value::Object;
		i++;
		ws();
		if (i < t.size() && t[i] == '}') {
			i++;
			return true;
		}
		for (;;) {
			ws();
			if (i >= t.size() || t[i] != '"')
				return fail("expected key");
			std::string key;
			if (!string(key))
				return false;
			ws();
			if (i >= t.size() || t[i] != ':')
				return fail("expected :");
			i++;
			Value e;
			if (!value(e))
				return false;
			v.o.emplace_back(std::move(key), std::move(e));
			ws();
			if (i < t.size() && t[i] == ',') {
				i++;
				ws();
				if (i < t.size() && t[i] == '}') {
					i++;
					return true;
				}
				continue;
			}
			if (i < t.size() && t[i] == '}') {
				i++;
				return true;
			}
			return fail("expected , or }");
		}
	}
};

}  // namespace

Value parse(const std::string& text, std::string* err) {
	Parser p(text);
	Value v;
	if (!p.value(v)) {
		if (err)
			*err = p.err;
		return Value();
	}
	if (err)
		err->clear();
	return v;
}

bool parse_file(const std::string& path, Value* out, std::string* err) {
	std::ifstream f(path, std::ios::binary);
	if (!f) {
		if (err)
			*err = "cannot open " + path;
		return false;
	}
	std::stringstream ss;
	ss << f.rdbuf();
	std::string e;
	*out = parse(ss.str(), &e);
	if (!e.empty()) {
		if (err)
			*err = path + ": " + e;
		return false;
	}
	return true;
}

}  // namespace json

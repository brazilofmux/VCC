/*
Copyright 2026 by the VCC Project Contributors.
This file is part of VCC (Virtual Color Computer).

    VCC (Virtual Color Computer) is free software: you can redistribute it
    and/or modify it under the terms of the GNU General Public License as
    published by the Free Software Foundation, either version 3 of the
    License, or (at your option) any later version.

    VCC (Virtual Color Computer) is distributed in the hope that it will be
    useful, but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License along
    with VCC (Virtual Color Computer). If not, see
    <http://www.gnu.org/licenses/>.
*/

// Portable implementation of the Win32 private-profile (INI) API,
// declared in vcc/util/host_services.h. Same file format and the
// semantics VCC relies on, so a vcc.ini written on Windows loads
// unchanged here:
//   - section/key lookup is case-insensitive, file case is preserved
//   - values have surrounding whitespace and one pair of quotes stripped
//   - writing a null value deletes the key; a null key deletes the section
//   - each write does a read-modify-write of the whole file (config
//     saves are rare and the file is small; simplicity wins)
//
// The enumeration forms (null app/key on read) are not implemented -
// nothing in the portable core uses them.
//
// Compiled only by the CMake build; on Windows the real API is used.
#ifndef _WIN32

#include <vcc/util/host_services.h>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <string>
#include <vector>

namespace
{
	bool iequals(const std::string& a, const std::string& b)
	{
		if (a.size() != b.size())
			return false;
		for (size_t i = 0; i < a.size(); ++i)
		{
			if (std::tolower((unsigned char)a[i]) != std::tolower((unsigned char)b[i]))
				return false;
		}
		return true;
	}

	std::string trim(const std::string& s)
	{
		size_t begin = 0;
		size_t end = s.size();
		while (begin < end && std::isspace((unsigned char)s[begin])) ++begin;
		while (end > begin && std::isspace((unsigned char)s[end - 1])) --end;
		return s.substr(begin, end - begin);
	}

	// "[Name]" -> "Name", or empty if the line is not a section header.
	std::string section_name(const std::string& line)
	{
		const std::string t = trim(line);
		if (t.size() < 2 || t.front() != '[' || t.back() != ']')
			return {};
		return trim(t.substr(1, t.size() - 2));
	}

	// "Key = value" -> "Key", or empty for comments/blank/section lines.
	std::string key_name(const std::string& line)
	{
		const std::string t = trim(line);
		if (t.empty() || t.front() == ';' || t.front() == '#' || t.front() == '[')
			return {};
		const size_t eq = t.find('=');
		if (eq == std::string::npos)
			return {};
		return trim(t.substr(0, eq));
	}

	std::string value_part(const std::string& line)
	{
		const size_t eq = line.find('=');
		std::string v = (eq == std::string::npos) ? std::string{} : trim(line.substr(eq + 1));
		if (v.size() >= 2 &&
		    ((v.front() == '"' && v.back() == '"') ||
		     (v.front() == '\'' && v.back() == '\'')))
		{
			v = v.substr(1, v.size() - 2);
		}
		return v;
	}

	std::vector<std::string> load_lines(const char* file)
	{
		std::vector<std::string> lines;
		std::ifstream in(file);
		std::string line;
		while (std::getline(in, line))
		{
			if (!line.empty() && line.back() == '\r')
				line.pop_back();
			lines.push_back(line);
		}
		return lines;
	}

	bool save_lines(const char* file, const std::vector<std::string>& lines)
	{
		std::ofstream out(file, std::ios::trunc);
		if (!out)
			return false;
		for (const auto& line : lines)
			out << line << '\n';
		return static_cast<bool>(out);
	}

	// Index of the "[app]" line, or npos.
	size_t find_section(const std::vector<std::string>& lines, const char* app)
	{
		for (size_t i = 0; i < lines.size(); ++i)
		{
			const std::string name = section_name(lines[i]);
			if (!name.empty() && iequals(name, app))
				return i;
		}
		return std::string::npos;
	}

	// Index of "key=" within the section starting at section_index, or npos.
	// Stops at the next section header.
	size_t find_key(const std::vector<std::string>& lines, size_t section_index,
	                const char* key)
	{
		for (size_t i = section_index + 1; i < lines.size(); ++i)
		{
			if (!section_name(lines[i]).empty())
				break;
			const std::string k = key_name(lines[i]);
			if (!k.empty() && iequals(k, key))
				return i;
		}
		return std::string::npos;
	}
}

DWORD GetPrivateProfileStringA(const char* app, const char* key,
                               const char* def, char* out, DWORD size,
                               const char* file)
{
	if (out == nullptr || size == 0)
		return 0;

	std::string value = def ? def : "";
	if (app != nullptr && key != nullptr && file != nullptr)
	{
		const auto lines = load_lines(file);
		const size_t section = find_section(lines, app);
		if (section != std::string::npos)
		{
			const size_t line = find_key(lines, section, key);
			if (line != std::string::npos)
				value = value_part(lines[line]);
		}
	}

	if (value.size() >= size)
		value.resize(size - 1);
	std::memcpy(out, value.c_str(), value.size() + 1);
	return (DWORD)value.size();
}

UINT GetPrivateProfileIntA(const char* app, const char* key, int def,
                           const char* file)
{
	char buf[64];
	const DWORD len = GetPrivateProfileStringA(app, key, "", buf, sizeof(buf), file);
	if (len == 0)
		return (UINT)def;
	return (UINT)std::strtol(buf, nullptr, 10);
}

BOOL WritePrivateProfileStringA(const char* app, const char* key,
                                const char* value, const char* file)
{
	if (app == nullptr || file == nullptr)
		return FALSE;

	auto lines = load_lines(file);
	size_t section = find_section(lines, app);

	if (key == nullptr)
	{
		// Delete the whole section.
		if (section == std::string::npos)
			return TRUE;
		size_t end = section + 1;
		while (end < lines.size() && section_name(lines[end]).empty())
			++end;
		lines.erase(lines.begin() + section, lines.begin() + end);
		return save_lines(file, lines);
	}

	if (section == std::string::npos)
	{
		if (value == nullptr)
			return TRUE;
		if (!lines.empty() && !trim(lines.back()).empty())
			lines.push_back("");
		lines.push_back(std::string("[") + app + "]");
		lines.push_back(std::string(key) + "=" + value);
		return save_lines(file, lines);
	}

	const size_t line = find_key(lines, section, key);
	if (value == nullptr)
	{
		// Delete the key.
		if (line == std::string::npos)
			return TRUE;
		lines.erase(lines.begin() + line);
		return save_lines(file, lines);
	}

	const std::string entry = std::string(key) + "=" + value;
	if (line != std::string::npos)
	{
		lines[line] = entry;
	}
	else
	{
		// Insert at the end of the section, before trailing blank lines.
		size_t end = section + 1;
		while (end < lines.size() && section_name(lines[end]).empty())
			++end;
		while (end > section + 1 && trim(lines[end - 1]).empty())
			--end;
		lines.insert(lines.begin() + end, entry);
	}
	return save_lines(file, lines);
}

#endif // !_WIN32

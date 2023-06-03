
/**
 * Copyright(C) 2019 Rokae Technology Co., Ltd.
 * All Rights Reserved.
 *
 * Information in this file is the intellectual property of Rokae Technology Co., Ltd,
 * And may contains trade secrets that must be stored and viewed confidentially.
 */
#pragma once
#include <map>
#include <string>

class INIParser {
   public:
    ~INIParser() { Clear(); }
    size_t GetSize() const { return _map_ini.size(); }
    std::string getStringFromFile(const std::string& file);
    bool ReadINI(const std::string& fileName);
    std::string GetString(const std::string& root, const std::string& key, const std::string& def = "") const;
    int GetInt(const std::string& root, const std::string& key, int def = 0) const;
    double GetDouble(const std::string& root, const std::string& key, double def = 0) const;
    bool GetBool(const std::string& root, const std::string& key, bool def = false) const;

    void SetValue(const std::string& root, const std::string& key, const std::string& value);
    void SetString(const std::string& root, const std::string& key, const std::string& value);
    void SetInt(const std::string& root, const std::string& key, int value);
    void SetDouble(const std::string& root, const std::string& key, double value);
    void SetBool(const std::string& root, const std::string& key, bool value);

    bool WriteINI(const std::string& path);
    void Clear() { _map_ini.clear(); }
    void View();

   private:
    std::map<std::string, std::map<std::string, std::string>> _map_ini;

    std::string& replace_all(std::string& str, const std::string& old_value, const std::string& new_value);
    std::string& replace_all_distinct(std::string& str, const std::string& old_value, const std::string& new_value);
};

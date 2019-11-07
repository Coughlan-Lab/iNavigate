//
//  ParseUtils.hpp
//  LocalizationCore
//
//  Created by Giovanni Fusco on 11/4/18.
//  Copyright © 2018 SKERI. All rights reserved.
//

#ifndef ParseUtils_h
#define ParseUtils_h

#include <map>
#include <iostream>
#include <stdlib.h>

#include <algorithm>
#include <cctype>
#include <locale>

namespace parseutils{
    
    using KeyValuePair = std::pair<std::string, std::string>;
    using IndexValuePair = std::pair<int, std::string>;
    
    //** String trimming functions from https://stackoverflow.com/a/217605
    
    // trim from start (in place)
    static inline void ltrim(std::string &s) {
        s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](int ch) {
            return !std::isspace(ch);
        }));
    }
    
    // trim from end (in place)
    static inline void rtrim(std::string &s) {
        s.erase(std::find_if(s.rbegin(), s.rend(), [](int ch) {
            return !std::isspace(ch);
        }).base(), s.end());
    }
    
    // trim from both ends (in place)
    static inline void trim(std::string &s) {
        ltrim(s);
        rtrim(s);
    }
    
    // trim from start (copying)
    static inline std::string ltrim_copy(std::string s) {
        ltrim(s);
        return s;
    }
    
    // trim from end (copying)
    static inline std::string rtrim_copy(std::string s) {
        rtrim(s);
        return s;
    }
    
    // trim from both ends (copying)
    static inline std::string trim_copy(std::string s) {
        trim(s);
        return s;
    }
    
    static KeyValuePair splitKeyValue(std::string strLine, std::string delimiter){
        
        assert(strLine.length() > 0);
        
        KeyValuePair kv;
        std::size_t found = strLine.find(delimiter);
        std::string tag = strLine.substr(0, found);
        std::string value = strLine.substr(found+1);
        kv = std::make_pair(ltrim_copy(tag), trim_copy(value));
        return kv;
    }
    
    static IndexValuePair splitIndexValue(std::string strLine, std::string delimiter){
        
        IndexValuePair kv;
        if(strLine.length() > 0){
            std::size_t found = strLine.find(delimiter);
            std::string tag = strLine.substr(0, found);
            std::string value = strLine.substr(found+1);
            kv = std::make_pair(stoi(ltrim_copy(tag)), trim_copy(value));
            
        }
        return kv;
    }
    
    template <typename T>
    static std::vector<T> parseCSVArray(std::string strLine){
        // remove padding brackets '[ ]'
        assert(strLine.length() > 2);
        strLine = strLine.substr(1, strLine.length()-2);
        std::vector<T> vect;
        std::stringstream ss(strLine);
        T item;
        while (ss >> item){
            vect.push_back(item);
            if ( (ss.peek() == ',') || (ss.peek() == ' ') )
                ss.ignore();
        }
        return vect;
    }
    
    template <typename T>
    static std::vector<T> parseCSVList(std::string strLine){
        assert(strLine.length() > 0);
        std::vector<T> vect;
        std::stringstream ss(strLine);
        T item;
        while (ss >> item){
            vect.push_back(item);
            if ( (ss.peek() == ',') || (ss.peek() == ' ') )
                ss.ignore();
        }
        return vect;
    }
    
    template <typename T>
    static std::vector<T> parseCSVListWithType(std::string strLine){
        assert(strLine.length() > 0);
        
        std::size_t found = strLine.find('(');
        strLine = strLine.substr(found+1, strLine.length()-3);
        std::vector<T> vect;
        std::stringstream ss(strLine);
        T item;
        while (ss >> item){
            vect.push_back(item);
            if ( (ss.peek() == ',') || (ss.peek() == ' ') )
                ss.ignore();
        }
        return vect;
    }
    
    
    
    
} // ::utils

#endif /* ParseUtils_h */

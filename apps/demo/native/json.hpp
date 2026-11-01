#pragma once
// Minimal strictly typed, bounded JSON for offline SLAM replay and sidecars.
// No network, dependency, unsafe JSON "number via double" fingerprint coercion.
#include <charconv>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace slam::viewer {
struct Json {
  enum class Kind { null, boolean, number, string, array, object };
  Kind kind{Kind::null};
  std::string scalar;
  std::vector<Json> array;
  std::map<std::string, Json> object;
  static Json boolean(bool x) { Json v; v.kind=Kind::boolean; v.scalar=x?"true":"false"; return v; }
  static Json number(std::string x) { Json v; v.kind=Kind::number; v.scalar=std::move(x); return v; }
  static Json text(std::string x) { Json v; v.kind=Kind::string; v.scalar=std::move(x); return v; }
  static Json list() { Json v; v.kind=Kind::array; return v; }
  static Json dict() { Json v; v.kind=Kind::object; return v; }
  const Json& at(std::string_view key) const {
    if(kind!=Kind::object) throw std::invalid_argument("expected JSON object");
    auto it=object.find(std::string(key));
    if(it==object.end()) throw std::invalid_argument("missing JSON member");
    return it->second;
  }
  const Json& at(std::size_t i) const {
    if(kind!=Kind::array || i>=array.size()) throw std::invalid_argument("JSON array bounds");
    return array[i];
  }
  bool has(std::string_view key) const {
    return kind==Kind::object && object.contains(std::string(key));
  }
  std::string str() const {
    if(kind!=Kind::string) throw std::invalid_argument("expected JSON string");
    return scalar;
  }
  bool flag() const {
    if(kind!=Kind::boolean) throw std::invalid_argument("expected JSON boolean");
    return scalar=="true";
  }
  double real() const {
    if(kind!=Kind::number) throw std::invalid_argument("expected JSON number");
    double n{};
    auto [end,err]=std::from_chars(scalar.data(),scalar.data()+scalar.size(),n);
    if(err!=std::errc{}||end!=scalar.data()+scalar.size()||!std::isfinite(n))
      throw std::invalid_argument("invalid finite JSON number");
    return n;
  }
  std::uint64_t uint() const {
    if(kind!=Kind::number || scalar.empty() || scalar[0]=='-' ||
       (scalar.size()>1 && scalar[0]=='0'))
      throw std::invalid_argument("expected canonical unsigned JSON integer");
    std::uint64_t v{};
    const auto [end,err]=std::from_chars(scalar.data(),scalar.data()+scalar.size(),v);
    if(err!=std::errc{}||end!=scalar.data()+scalar.size())
      throw std::invalid_argument("JSON integer overflow/not integer");
    return v;
  }
  std::string dump() const;
};

inline std::string quote_json(std::string_view s) {
  std::string r="\"";
  constexpr char hex[]="0123456789abcdef";
  for(unsigned char c:s) {
    if(c=='"'||c=='\\') {r+='\\';r+=static_cast<char>(c);}
    else if(c<32) {r+="\\u00";r+=hex[c>>4];r+=hex[c&15];}
    else r+=static_cast<char>(c);
  }
  return r+"\"";
}
inline std::string Json::dump() const {
  if(kind==Kind::string) return quote_json(scalar);
  if(kind==Kind::number||kind==Kind::boolean) return scalar;
  if(kind==Kind::null) return "null";
  std::string out=kind==Kind::array?"[":"{";
  bool first=true;
  if(kind==Kind::array) for(const auto& entry:array) {
    if(!first)out+=",";
    first=false;out+=entry.dump();
  }
  else for(const auto& [key,entry]:object) {
    if(!first)out+=",";
    first=false;out+=quote_json(key)+":"+entry.dump();
  }
  return out+(kind==Kind::array?"]":"}");
}
class JsonParser {
public:
  explicit JsonParser(std::string_view input): s_(input) {
    if(input.size()>32*1024*1024)throw std::invalid_argument("oversized JSON");
  }
  Json parse() {
    auto v=value(0);space();
    if(pos_!=s_.size())throw std::invalid_argument("trailing JSON data");
    return v;
  }
private:
  std::string_view s_;
  std::size_t pos_{};
  void space() {while(pos_<s_.size()&&(s_[pos_]==' '||s_[pos_]=='\n'||
      s_[pos_]=='\r'||s_[pos_]=='\t'))++pos_;}
  bool take(char c) {space();if(pos_<s_.size()&&s_[pos_]==c){++pos_;return true;}return false;}
  char next() {if(pos_==s_.size())throw std::invalid_argument("truncated JSON");return s_[pos_++];}
  static void utf8(std::string& out,unsigned cp) {
    if(cp<=0x7f)out+=static_cast<char>(cp);
    else if(cp<=0x7ff){out+=static_cast<char>(0xc0|(cp>>6));out+=static_cast<char>(0x80|(cp&63));}
    else if(cp<=0xffff){out+=static_cast<char>(0xe0|(cp>>12));out+=static_cast<char>(0x80|((cp>>6)&63));out+=static_cast<char>(0x80|(cp&63));}
    else {out+=static_cast<char>(0xf0|(cp>>18));out+=static_cast<char>(0x80|((cp>>12)&63));out+=static_cast<char>(0x80|((cp>>6)&63));out+=static_cast<char>(0x80|(cp&63));}
  }
  unsigned hex4() {
    unsigned n=0;for(int i=0;i<4;++i){const char c=next();n<<=4;
      if(c>='0'&&c<='9')n+=static_cast<unsigned>(c-'0');
      else if(c>='a'&&c<='f')n+=static_cast<unsigned>(c-'a'+10);
      else if(c>='A'&&c<='F')n+=static_cast<unsigned>(c-'A'+10);
      else throw std::invalid_argument("invalid JSON Unicode escape");
    }return n;
  }
  std::string string() {
    if(next()!='"')throw std::invalid_argument("expected JSON string");
    std::string out;
    while(true) {
      const unsigned char c=static_cast<unsigned char>(next());
      if(c=='"')break;
      if(c<32)throw std::invalid_argument("JSON control character");
      if(c!='\\') {out+=static_cast<char>(c);continue;}
      const char e=next();
      if(e=='"'||e=='\\'||e=='/')out+=e;
      else if(e=='b')out+='\b';
      else if(e=='f')out+='\f';
      else if(e=='n')out+='\n';
      else if(e=='r')out+='\r';
      else if(e=='t')out+='\t';
      else if(e=='u') {
        unsigned cp=hex4();
        if(cp>=0xd800 && cp<=0xdbff) {
          if(next()!='\\'||next()!='u')throw std::invalid_argument("JSON surrogate pair missing");
          const unsigned low=hex4();
          if(low<0xdc00||low>0xdfff)throw std::invalid_argument("invalid JSON surrogate");
          cp=0x10000+((cp-0xd800)<<10)+(low-0xdc00);
        } else if(cp>=0xdc00&&cp<=0xdfff)throw std::invalid_argument("orphan JSON surrogate");
        utf8(out,cp);
      } else throw std::invalid_argument("invalid JSON escape");
    }
    return out;
  }
  Json value(unsigned depth) {
    if(depth>32)throw std::invalid_argument("JSON nesting exceeded");
    space();if(pos_>=s_.size())throw std::invalid_argument("truncated JSON value");
    if(s_[pos_]=='"')return Json::text(string());
    if(take('{')) {
      Json out=Json::dict();if(take('}'))return out;
      do{
        space();const std::string key=string();
        if(!take(':'))throw std::invalid_argument("missing JSON colon");
        Json v=value(depth+1);
        if(!out.object.emplace(key,std::move(v)).second)
          throw std::invalid_argument("duplicate JSON object key");
        if(take('}'))return out;
        if(!take(','))throw std::invalid_argument("missing JSON object comma");
      }while(true);
    }
    if(take('[')) {
      Json out=Json::list();if(take(']'))return out;
      do{
        out.array.push_back(value(depth+1));
        if(take(']'))return out;
        if(!take(','))throw std::invalid_argument("missing JSON array comma");
      }while(true);
    }
    for(auto literal:{"true","false","null"}) {
      const std::string_view word(literal);
      if(s_.substr(pos_,word.size())==word) {
        pos_+=word.size();
        if(word=="null")return Json{};
        return Json::boolean(word=="true");
      }
    }
    const auto start=pos_;
    if(s_[pos_]=='-')++pos_;
    if(pos_>=s_.size())throw std::invalid_argument("truncated JSON number");
    if(s_[pos_]=='0')++pos_;
    else {
      if(s_[pos_]<'1'||s_[pos_]>'9')throw std::invalid_argument("invalid JSON number");
      while(pos_<s_.size()&&s_[pos_]>='0'&&s_[pos_]<='9')++pos_;
    }
    if(pos_<s_.size()&&s_[pos_]=='.') {
      ++pos_;const auto p=pos_;
      while(pos_<s_.size()&&s_[pos_]>='0'&&s_[pos_]<='9')++pos_;
      if(pos_==p)throw std::invalid_argument("invalid JSON fraction");
    }
    if(pos_<s_.size()&&(s_[pos_]=='e'||s_[pos_]=='E')) {
      ++pos_;if(pos_<s_.size()&&(s_[pos_]=='+'||s_[pos_]=='-'))++pos_;
      const auto p=pos_;
      while(pos_<s_.size()&&s_[pos_]>='0'&&s_[pos_]<='9')++pos_;
      if(pos_==p)throw std::invalid_argument("invalid JSON exponent");
    }
    return Json::number(std::string(s_.substr(start,pos_-start)));
  }
};
inline Json parse_json(std::string_view input) {return JsonParser(input).parse();}
inline Json jint(std::uint64_t n){return Json::number(std::to_string(n));}
inline Json jreal(double n){if(!std::isfinite(n))throw std::invalid_argument("non-finite JSON");return Json::number(std::to_string(n));}
inline void need(bool yes,const char* why){if(!yes)throw std::invalid_argument(why);}
inline bool keys(const Json& v,std::initializer_list<std::string_view> names) {
  if(v.kind!=Json::Kind::object||v.object.size()!=names.size())return false;
  for(const auto& n:names)if(!v.has(n))return false;
  return true;
}
} // namespace slam::viewer

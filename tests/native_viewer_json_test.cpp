#include "apps/demo/native/json.hpp"

#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <string_view>

namespace {
using slam::viewer::Json;
using slam::viewer::parse_json;

void require(bool condition, std::string_view message) {
  if (!condition) {
    std::cerr << "native JSON regression failed: " << message << '\n';
    std::exit(1);
  }
}

void rejects(std::string_view source) {
  bool refused = false;
  try {
    (void)parse_json(source);
  } catch (const std::invalid_argument&) {
    refused = true;
  }
  require(refused, "malformed JSON was accepted");
}
} // namespace

int main() {
  // These identifiers exceed binary64's exact integer range. They must
  // survive parsing and serialization without implicit double conversion.
  const auto signed_report = parse_json(
      R"({"fingerprint":"18446744073709551615","revision":"9007199254740993","ok":true})");
  require(signed_report.at("fingerprint").str() == "18446744073709551615",
          "map fingerprint changed");
  require(signed_report.at("revision").str() == "9007199254740993",
          "map revision changed");
  require(signed_report.at("ok").flag(), "boolean source changed");
  require(parse_json(signed_report.dump()).dump() == signed_report.dump(),
          "JSON roundtrip not deterministic");

  const auto numbers = parse_json(R"({"id":18446744073709551615,"neg":-0.5,"null":null})");
  require(numbers.at("id").uint() == 18446744073709551615ULL,
          "uint64 identity was rounded or truncated");
  require(numbers.at("neg").real() == -0.5, "float decoding changed");
  require(numbers.at("null").kind == Json::Kind::null, "null changed");

  const auto nested = parse_json(R"({"a":[1,{"b":"quoted\"backslash\\"},false]})");
  require(nested.at("a").at(1).at("b").str() == "quoted\"backslash\\",
          "escaped string encoding changed");

  // Strict parser must refuse duplicate keys and false provenance fields.
  rejects(R"({"a":1,"a":2})");
  rejects(R"({"a":true,"a":false})");
  rejects(R"({"f":NaN})");
  rejects(R"({"f":Infinity})");
  rejects(R"({"f":1e999999})");
  rejects(R"({"f":01})");
  rejects(R"({"f":1.})");
  rejects(R"({"f":1e})");
  rejects(R"({"f":+1})");
  rejects(R"({"s":"\uD800"})");
  rejects(R"({"s":"\uDC00"})");
  rejects(R"({"s":"\x00"})");
  rejects(R"([true] garbage)");
  rejects(R"({"unfinished":)");
  rejects("[]]");

  bool bad_uint = false;
  try {
    (void)parse_json(R"({"id":1.5})").at("id").uint();
  } catch (const std::invalid_argument&) {
    bad_uint = true;
  }
  require(bad_uint, "fractional fingerprint accepted as exact unsigned integer");

  const auto encoded = slam::viewer::jreal(0.12345678901234566);
  require(parse_json(encoded.dump()).real() == 0.12345678901234566,
          "binary64 JSON encoding lost precision");

  std::cout << "strict native viewer JSON parsing and 64-bit identity passed\n";
}

#include "ults.h"
#include <cmath>

namespace MNM_Ults
{
void
set_random_state (unsigned int s)
{
  std::srand (s);
}

TInt
round (TFlt in)
{
  TFlt rdNum = TFlt (std::rand () / (1.0 * RAND_MAX));
  TFlt floorN = TFlt (TInt (in));
  if ((in - floorN) > rdNum)
    return TInt (floorN + 1);
  else
    return TInt (floorN);
}

TFlt
divide (TFlt a, TFlt b)
{
  if (a == TFlt (0))
    return TFlt (0);
  if (b == TFlt (0))
    return TFlt (0);
  return a / b;
}

TInt
mod (TInt a, TInt b)
{
  if (b == TInt (0))
    return TInt (0);
  return a % b;
}

TFlt
rand_flt ()
{
  return TFlt ((double) std::rand () / (RAND_MAX));
}

TFlt
max_link_cost ()
{
  return TFlt (60 * 60);
}

int
copy_file (const char *srce_file, const char *dest_file)
{
  std::ifstream srce (srce_file, std::ios::binary);
  std::ofstream dest (dest_file, std::ios::binary);
  dest << srce.rdbuf ();
  return 0;
}

int
copy_file (std::string srce_file, std::string dest_file)
{
  return MNM_Ults::copy_file (srce_file.c_str (), dest_file.c_str ());
}

TFlt
roundoff (TFlt value, unsigned char prec)
{
  TFlt pow_10 = pow (10.0f, (TFlt) prec);
  return round (value * pow_10) / pow_10;
}

bool
approximate_equal (TFlt a, TFlt b, TFlt p)
{
  // approximately equal,
  // https://stackoverflow.com/questions/17333/what-is-the-most-effective-way-for-float-and-double-comparison
  if (fabs (a - b) <= p * std::max (fabs (a), fabs (b)))
    {
      return true;
    }
  else
    {
      return false;
    }
}

bool
approximate_less_than (TFlt a, TFlt b, TFlt p)
{
  return a + p * std::max (fabs (a), fabs (b)) < b;
}

int
round_up_time (TFlt time, TFlt p)
{
  if (time < 1)
    {
      return 1;
    }
  else
    {
      IAssert (int (time) >= 1);
      if (approximate_equal (time, TFlt (int (time)), p))
        {
          return int (time);
        }
      else
        {
          IAssert (int (time) + 1 > time);
          return int (time) + 1;
        }
    }
}

int
round_down_time (TFlt time)
{
  if (time < 1)
    {
      return 1;
    }
  else
    {
      IAssert (int (time) >= 1);
      return int (time);
    }
}

macposts::Graph
reverse_graph (const macposts::Graph &graph)
{
  macposts::Graph reversed_graph;
  for (const auto &node : graph.nodes ())
    reversed_graph.add_node (graph.get_id (node));
  for (const auto &link : graph.links ())
    {
      auto endpoints = graph.get_endpoints (link);
      auto from = graph.get_id (endpoints.first);
      auto to = graph.get_id (endpoints.second);
      auto id = graph.get_id (link);
      reversed_graph.add_link (to, from, id);
    }
  return reversed_graph;
}
}

Chameleon::Chameleon (std::string const &value) { value_ = value; }

Chameleon::Chameleon (const char *c) { value_ = c; }

Chameleon::Chameleon (double d)
{
  std::stringstream s;
  s << d;
  value_ = s.str ();
}

Chameleon::Chameleon (Chameleon const &other) { value_ = other.value_; }

Chameleon &
Chameleon::operator= (Chameleon const &other)
{
  value_ = other.value_;
  return *this;
}

Chameleon &
Chameleon::operator= (double i)
{
  std::stringstream s;
  s << i;
  value_ = s.str ();
  return *this;
}

Chameleon &
Chameleon::operator= (std::string const &s)
{
  value_ = s;
  return *this;
}

Chameleon::operator std::string () const { return value_; }

Chameleon::operator double () const { return atof (value_.c_str ()); }

std::string
trim (std::string const &source, char const *delims = " \t\r\n")
{
  std::string result (source);
  std::string::size_type index = result.find_last_not_of (delims);
  if (index != std::string::npos)
    result.erase (++index);

  index = result.find_first_not_of (delims);
  if (index != std::string::npos)
    result.erase (0, index);
  else
    result.erase ();
  return result;
}

ConfigFile::ConfigFile (std::string const &configFile)
{
  std::ifstream file (configFile.c_str ());

  std::string line;
  std::string name;
  std::string value;
  std::string inSection;
  int posEqual;
  while (std::getline (file, line))
    {
      line = trim (line);
      if (!line.length ())
        continue;

      switch (line[0])
        {
        case '#':
        case ';':
          continue;
        case '[':
          inSection = trim (line.substr (1, line.find (']') - 1));
          continue;
        default:
          posEqual = line.find ('=');
          name = trim (line.substr (0, posEqual));
          value = trim (line.substr (posEqual + 1));
          content_[inSection + '/' + name] = Chameleon (value);
        }
    }
}

Chameleon const &
ConfigFile::Value (std::string const &section, std::string const &entry) const
{
  std::map<std::string, Chameleon>::const_iterator ci
    = content_.find (section + '/' + entry);

  if (ci == content_.end ())
    throw std::invalid_argument ("does not exist");

  return ci->second;
}

Chameleon const &
ConfigFile::Value (std::string const &section, std::string const &entry,
                   double value)
{
  try
    {
      return Value (section, entry);
    }
  catch (const char *)
    {
      return content_
        .insert (std::make_pair (section + '/' + entry, Chameleon (value)))
        .first->second;
    }
}

Chameleon const &
ConfigFile::Value (std::string const &section, std::string const &entry,
                   std::string const &value)
{
  try
    {
      return Value (section, entry);
    }
  catch (const char *)
    {
      return content_
        .insert (std::make_pair (section + '/' + entry, Chameleon (value)))
        .first->second;
    }
}

MNM_ConfReader::MNM_ConfReader (const std::string &filename,
                                std::string confKey)
{
  m_configFile = new ConfigFile (filename);
  m_confKey = confKey;
}

MNM_ConfReader::~MNM_ConfReader ()
{
  if (m_configFile != NULL)
    {
      delete m_configFile;
    }
}

TInt
MNM_ConfReader::get_int (const std::string &key)
{
  int val = m_configFile->Value (m_confKey, key);
  return TInt (val);
}

TFlt
MNM_ConfReader::get_float (const std::string &key)
{
  double val = m_configFile->Value (m_confKey, key);
  return TFlt (val);
}

std::string
MNM_ConfReader::get_string (const std::string &key)
{
  std::string val = m_configFile->Value (m_confKey, key);
  return val;
}

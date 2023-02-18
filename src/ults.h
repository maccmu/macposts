#ifndef ULTS_H
#define ULTS_H

#include <map>
#include <string>

#include <Snap.h>

template <> struct std::hash<TInt>
{
  std::size_t
  operator() (const TInt &s) const
  {
    return std::hash<int> () (s ());
  }
};

namespace MNM_Ults
{
void srand (unsigned int s);
TInt round (TFlt in);
TFlt min (TFlt a, TFlt b);
TInt min (TInt a, TInt b);
TFlt max (TFlt a, TFlt b);
TFlt divide (TFlt a, TFlt b);
TInt mod (TInt a, TInt b);
TFlt rand_flt ();
TFlt max_link_cost ();
int copy_file (const char *srce_file, const char *dest_file);
int copy_file (std::string srce_file, std::string dest_file);
}

class Chameleon
{
public:
  Chameleon (){};
  explicit Chameleon (const std::string &);
  explicit Chameleon (double);
  explicit Chameleon (const char *);

  ~Chameleon (){};

  Chameleon (const Chameleon &);
  Chameleon &operator= (Chameleon const &);

  Chameleon &operator= (double);
  Chameleon &operator= (std::string const &);

public:
  operator std::string () const;
  operator double () const;

private:
  std::string value_;
};

class ConfigFile
{
  std::map<std::string, Chameleon> content_;

public:
  ConfigFile (std::string const &configFile);
  ~ConfigFile (){};

  Chameleon const &Value (std::string const &section,
                          std::string const &entry) const;

  Chameleon const &Value (std::string const &section, std::string const &entry,
                          double value);
  Chameleon const &Value (std::string const &section, std::string const &entry,
                          std::string const &value);
};

class MNM_ConfReader
{
public:
  MNM_ConfReader (std::string, std::string);
  ~MNM_ConfReader ();

  TInt get_int (std::string);
  std::string get_string (std::string);
  TFlt get_float (std::string);

  /* data */
  ConfigFile *m_configFile;
  std::string m_confKey;
};

#endif

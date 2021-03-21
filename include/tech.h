#ifndef TECH_H
#define TECH_H

#include <list>

#include <lefrReader.hpp>

#include "layer.h"
#include "site.h"
#include "macro.h"
#include "lefvia.h"
#include "viarulegenerate.h"

namespace phydb {

class Tech {
    friend class PhyDB;
  private:
    string version_;
    string bus_bit_char_;
    string divier_char_;
    string clearance_measure_;
    double manufacturing_grid_;

    int dbu_per_micron_;

    vector<Site> sites_;
    vector<Layer> layers_;
    vector<Macro> macros_;
    vector<LefVia> vias_;
    vector<ViaRuleGenerate> via_rule_generates_;

    map<string, int> layer_2_id_;
    map<string, int> macro_2_id_;
    map<string, int> via_2_id_;
    map<string, int> via_rule_generate_2_id_;

    /****placement grid parameters****/
    bool is_placement_grid_set_ = false;
    double placement_grid_value_x_ = 0;
    double placement_grid_value_y_ = 0;

    /****N/P-well layers_****/
    bool is_n_well_layer_set_ = false;
    bool is_p_well_layer_set_ = false;
    WellLayer *n_layer_ptr_ = nullptr;
    WellLayer *p_layer_ptr_ = nullptr;
    double same_diff_spacing_ = -1;
    double any_diff_spacing_ = -1;
    std::list<MacroWell> wells_;

  public:
    Tech() : manufacturing_grid_(0), dbu_per_micron_(0) {}

    void SetDatabaseMicron(int database_micron);
    void SetManufacturingGrid(double manufacture_grid);
    void AddSite(std::string const &name, std::string const &class_name, double width, double height);
    void SetPlacementGrids(double placement_grid_value_x, double placement_grid_value_y);

    bool IsLayerExist(std::string const &layer_name);
    Layer *AddLayer(std::string &layer_name);
    Layer *GetLayerPtr(std::string const &layer_name);
    int GetLayerId(const string& layer_name);
    void ReportLayers();

    bool IsMacroExist(std::string const &macro_name);
    Macro *AddMacro(std::string &macro_name);
    Macro *GetMacroPtr(std::string const &macro_name);

    bool IsLefViaExist(std::string const &via_name);
    LefVia *AddLefVia(std::string &via_name);
    LefVia *GetLefViaPtr(std::string const &via_name);

    bool IsViaRuleGenerateExist(std::string const &name);
    ViaRuleGenerate* AddViaRuleGenerate(std::string &name);
    ViaRuleGenerate* GetViaRuleGeneratePtr(std::string const &name);
    
    void SetNwellLayer(double width, double spacing, double op_spacing, double max_plug_dist, double overhang);
    void SetPwellLayer(double width, double spacing, double op_spacing, double max_plug_dist, double overhang);
    void SetNpwellSpacing(double same_diff, double any_diff);
    bool IsWellInfoSet() const;
    void ReportWellShape(); // report the well shape_ for each BlockType for debugging purposes.


    friend ostream &operator<<(ostream &, const Tech &);

    friend int getLefSite(lefrCallbackType_e, lefiSite *, lefiUserData);
    friend int getLefMacros(lefrCallbackType_e, lefiMacro *, lefiUserData);
    friend int getLefString(lefrCallbackType_e, const char *, lefiUserData);
    friend int getLefUnits(lefrCallbackType_e, lefiUnits *, lefiUserData);
    friend int getLefManufacturingGrid(lefrCallbackType_e, double, lefiUserData);
    friend int getLefPins(lefrCallbackType_e, lefiPin *, lefiUserData);
    friend int getLefObs(lefrCallbackType_e, lefiObstruction *, lefiUserData);
    friend int getLefLayers(lefrCallbackType_e, lefiLayer *, lefiUserData);
    friend int getLefVias(lefrCallbackType_e, lefiVia *, lefiUserData);
    friend int getLefViaGenerateRules(lefrCallbackType_e, lefiViaRule *, lefiUserData);

};

int getLefSite(lefrCallbackType_e type, lefiSite *site, lefiUserData data);
int getLefMacrosBegin(lefrCallbackType_e type, const char *str, lefiUserData data);
int getLefMacros(lefrCallbackType_e, lefiMacro *, lefiUserData);
int getLefMacrosEnd(lefrCallbackType_e type, const char *str, lefiUserData data);
int getLefString(lefrCallbackType_e, const char *, lefiUserData);
int getLefUnits(lefrCallbackType_e, lefiUnits *, lefiUserData);
int getLefManufacturingGrid(lefrCallbackType_e, double, lefiUserData);
int getLefPins(lefrCallbackType_e, lefiPin *, lefiUserData);
int getLefObs(lefrCallbackType_e, lefiObstruction *, lefiUserData);
int getLefLayers(lefrCallbackType_e, lefiLayer *, lefiUserData);
int getLefVias(lefrCallbackType_e, lefiVia *, lefiUserData);
int getLefViaGenerateRules(lefrCallbackType_e, lefiViaRule *, lefiUserData);

ostream &operator<<(ostream &, const Tech &);

}

#endif


#ifndef LAYER_H
#define LAYER_H

#include "header.h"
#include "spacingtable.h"
#include "eolspacing.h"
#include "cornerspacing.h"
#include "adjacentcutspacing.h"
#include "spacingtableinfluence.h"
#include "enumtypes.h"

namespace phydb {

class Layer {
  private:
    string name_;
    LayerType type_;
    int idx_;

    //metal layer
    MetalDirection direction_;
    float pitchx_;
    float pitchy_;
    float width_;
    float area_;
    float min_width_;
    float offset_;

    SpacingTable spacing_table_;
    vector<SpacingTableInfluence> spacing_table_influences_;
    vector<EolSpacing> eol_spacings_;
    CornerSpacing corner_spacing_;

    //cut layer
    float spacing_;
    AdjacentCutSpacing adjacent_cut_spacing_;

  public:
    Layer() : name_(""), type_(ROUTING ), idx_(-1), direction_(HORIZONTAL), pitchx_(0),
              pitchy_(0), width_(0), area_(0), min_width_(0), offset_(0), spacing_(0) {}
    Layer(std::string &name, LayerType type, MetalDirection direction) :
        name_(name), type_(type), direction_(direction) {}

    //constructor for metal layer
    Layer(string name, LayerType type, MetalDirection direction, float pitchx, float pitchy,
          float width, float area, float minWidth, float offset) :
        name_(name),
        type_(type),
        idx_(-1),
        direction_(direction),
        pitchx_(pitchx),
        pitchy_(pitchy),
        width_(width),
        area_(area),
        min_width_(minWidth),
        offset_(offset) {}

    //constructor for cut layer
    Layer(string name, LayerType type, float spacing) :
        name_(name),
        type_(type),
        idx_(-1),
        spacing_(spacing) {}

    string GetName() const;

    void SetName(string &name);
    void SetType(LayerType type);
    void SetDirection(MetalDirection direction);
    void SetWidth(float width);
    void SetMinWidth(float min_width);
    void SetPitch(float pitch_x, float pitch_y);
    void SetOffset(float offset);
    void SetArea(float area);
    void SetSpacing(float spacing);

    SpacingTable *SetSpacingTable(SpacingTable &);
    SpacingTable *SetSpacingTable(int n_col, int n_row, const vector<float> &v_parallel_run_length,
                                  const vector<float> &v_width, const vector<float> &v_spacing);
    SpacingTableInfluence *AddSpacingTableInfluence(float width, float within, float spacing);

    EolSpacing *AddEolSpacing(float spacing, float eol_width, float eol_within,
                              float par_edge, float par_within);
    CornerSpacing *SetCornerSpacing(CornerSpacing &);
    AdjacentCutSpacing *SetAdjCutSpacing(float spacing, int adjacent_cuts, int cut_within);

    float GetSpacing() const;
    SpacingTable *GetSpacingTable();
    vector<SpacingTableInfluence> *GetSpacingTableInfluences();
    vector<EolSpacing> *GetEolSpacings();
    CornerSpacing *GetCornerSpacing();
    AdjacentCutSpacing *GetAdjCutSpacing();

    friend ostream &operator<<(ostream &, const Layer &);

    void Report();
};

ostream &operator<<(ostream &, const Layer &);

class WellLayer {
  private:
    double width_;
    double spacing_;
    double op_spacing_;
    double max_plug_dist_;
    double overhang_;
  public:
    WellLayer(double width, double spacing, double op_spacing, double max_plug_dist, double overhang) {
        SetWidth(width);
        SetSpacing(spacing);
        SetOpSpacing(op_spacing);
        SetMaxPlugDist(max_plug_dist);
        SetOverhang(overhang);
    }

    double GetWidth() const;
    double GetSpacing() const;
    double GetOpSpacing() const;
    double GetMaxPlugDist() const;
    double GetOverhang() const;

    void SetWidth(double width);
    void SetSpacing(double spacing);
    void SetOpSpacing(double op_spacing);
    void SetMaxPlugDist(double max_plug_dist);
    void SetOverhang(double overhang);
    void SetParams(double width, double spacing, double op_spacing, double max_plug_dist, double overhang);
    void Report() const;
};

}

#endif

#ifndef COMPONENT_H
#define COMPONENT_H

#include "datatype.h"
#include "enumtypes.h"
#include "logging.h"
#include "macro.h"

namespace phydb {

class Component {
  public:
    int id_;
    string name_;
    Macro *macro_ptr_;
    string source_;
    PlaceStatus place_status_;
    CompOrient orient_;

    int weight_;
    Point2D<int> location_;

    Component() {}
    Component(string name, Macro *macroName, string source, PlaceStatus place_status,
              CompOrient orient, int weight, Point2D<int> location) :
        name_(name),
        macro_ptr_(macroName),
        source_(source),
        place_status_(place_status),
        orient_(orient),
        weight_(weight),
        location_(location) {}
    Component(std::string &comp_name, Macro *macro_name, PlaceStatus place_status,
              int llx, int lly, CompOrient orient)
        : name_(comp_name), macro_ptr_(macro_name), place_status_(place_status), orient_(orient) {
        location_.x = llx;
        location_.y = lly;
    }

    void SetPlacementStatus(PlaceStatus status);
    void SetOrientation(CompOrient orient);
    void SetLocation(int lx, int ly);

    const string &GetName();
    Macro *GetMacro();
    PlaceStatus GetPlacementStatus();
    CompOrient GetOrientation();
    Point2D<int> GetLocation();
    int GetWeight() { return weight_; }
};

ostream &operator<<(ostream &, const Component &);

}

#endif


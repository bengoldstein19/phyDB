/*******************************************************************************
 *
 * Copyright (c) 2021 Jiayuan He, Yihang Yang
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor,
 * Boston, MA  02110-1301, USA.
 *
 ******************************************************************************/
#include "lefdefparser.h"

#include <algorithm>

#include "datatype.h"
#include "phydb/common/logging.h"

namespace phydb {

int getLefSite(lefrCallbackType_e type, lefiSite *site, lefiUserData data) {
  if (type != lefrSiteCbkType) {
    std::cout << "Type is not lefrSiteCbkType!" << std::endl;
    exit(2);
  }
  if (site->lefiSite::hasSize()) {
    auto *phy_db_ptr = (PhyDB *) data;
    std::string site_name(site->name());
    std::string site_class_name;
    if (site->hasClass()) {
      site_class_name = std::string(site->siteClass());
    }
    Site *phydb_site = phy_db_ptr->AddSite(
        site_name,
        site_class_name,
        site->sizeX(),
        site->sizeY()
    );
    phydb_site->SetSymmetry(
        site->hasXSymmetry(),
        site->hasYSymmetry(),
        site->has90Symmetry()
    );
  } else {
    PhyDBExpects(false, "SITE SIZE information not provided");
  }
  return 0;
}

int getLefVersion(lefrCallbackType_e type, double version, lefiUserData data) {
  if (type != lefrVersionCbkType) {
    std::cout << "Type is not lefrVersionCbkType!" << std::endl;
    exit(2);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  phy_db_ptr->SetLefVersion(version);
  return 0;
}

int getLefMacrosBegin(
    lefrCallbackType_e type,
    const char *str,
    lefiUserData data
) {
  if (type != lefrMacroBeginCbkType) {
    std::cout << "Type is not lefrMacroBeginCbkType!" << std::endl;
    exit(2);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  std::string tmpMacroName(str);
  phy_db_ptr->AddMacro(tmpMacroName); //add an empty macro

  return 0;
}

int getLefMacros(lefrCallbackType_e type, lefiMacro *macro, lefiUserData data) {
  if (type != lefrMacroCbkType) {
    std::cout << "Type is not lefrMacroCbkType!" << std::endl;
    exit(2);
  }

  double originX = macro->originX();
  double originY = macro->originY();
  PhyDBExpects(
      (originX == 0) && (originY == 0),
      "Nonzero origin is not supported, macro: " << macro->name()
  );
  double sizeX = macro->sizeX();
  double sizeY = macro->sizeY();

  PhyDBExpects(
      macro->hasClass(),
      "Macro has no class?" + std::string(macro->name())
  );
  std::string str_macro_class(macro->macroClass());
  MacroClass macro_class = StrToMacroClass(str_macro_class);

  auto *phy_db_ptr = (PhyDB *) data;
  //write to the last one
  Macro &phydb_macro = phy_db_ptr->GetTechPtr()->GetMacrosRef().back();
  phydb_macro.SetOrigin(originX, originY);
  phydb_macro.SetSize(sizeX, sizeY);
  phydb_macro.SetClass(macro_class);
  phydb_macro.SetSymmetry(
      macro->hasXSymmetry(),
      macro->hasYSymmetry(),
      macro->has90Symmetry()
  );
  phydb_macro.SetSite(macro->siteName());
  return 0;
}

int getLefMacrosEnd(
    lefrCallbackType_e type,
    const char *str,
    lefiUserData data
) {
  if (type != lefrMacroEndCbkType) {
    std::cout << "Type is not lefrMacroEndCbkType!" << std::endl;
    exit(1);
  }
  return 0;
}

int getLefUnits(lefrCallbackType_e type, lefiUnits *units, lefiUserData data) {
  if (type != lefrUnitsCbkType) {
    std::cout << "Type is not lefrUnitsCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;

  int database_unit = (int) units->databaseNumber();
  phy_db_ptr->SetDatabaseMicron(database_unit);

  return 0;
}

int getLefManufacturingGrid(
    lefrCallbackType_e type,
    double number,
    lefiUserData data
) {
  if (type != lefrManufacturingCbkType) {
    std::cout << "Type is not lefrManufacturingCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  phy_db_ptr->SetManufacturingGrid(number);
  return 0;
}

int getLefPins(lefrCallbackType_e type, lefiPin *pin, lefiUserData data) {
  auto *phy_db_ptr = (PhyDB *) data;
  if (type != lefrPinCbkType) {
    std::cout << "Type is not lefrPinCbkType!" << std::endl;
    exit(1);
  }

  // last macro
  Macro &last_macro = phy_db_ptr->GetTechPtr()->GetMacrosRef().back();

  std::string pin_name(pin->name());
  std::string pin_direction(pin->direction());
  std::string pin_use(pin->use());
  Pin *pin_ptr = last_macro.AddPin(
      pin_name,
      StrToSignalDirection(pin_direction),
      StrToSignalUse(pin_use)
  );

  int numPorts = pin->numPorts();
  if (numPorts <= 0) {
    PhyDBExpects(false, "No physical ports? Macro: " +
        last_macro.GetName() + ", pin: " + pin_name);
  }

  for (int i = 0; i < numPorts; ++i) {
    int numItems = pin->port(i)->numItems();
    LayerRect *layer_rect_ptr = nullptr;
    for (int j = 0; j < numItems; ++j) {
      int itemType = pin->port(i)->itemType(j);
      if (itemType == 1) { //layer
        std::string layer_name(pin->port(i)->getLayer(j));
        layer_rect_ptr = pin_ptr->AddLayerRect(layer_name);
      } else if (itemType == 8) {
        double x1 = pin->port(i)->getRect(j)->xl;
        double y1 = pin->port(i)->getRect(j)->yl;
        double x2 = pin->port(i)->getRect(j)->xh;
        double y2 = pin->port(i)->getRect(j)->yh;
        PhyDBExpects(layer_rect_ptr != nullptr, "unexpected error");
        layer_rect_ptr->AddRect(
            std::min(x1, x2),
            std::min(y1, y2),
            std::max(x1, x2),
            std::max(y1, y2)
        );
      } else {
        std::cout << "unsupported lefiGeometries!\n";
        continue;
      }
    }
  }
  return 0;
}

int getLefObs(
    lefrCallbackType_e type,
    lefiObstruction *obs,
    lefiUserData data
) {

  if (type != lefrObstructionCbkType) {
    std::cout << "Type is not lefrObstructionCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  Macro &last_macro =
      phy_db_ptr->GetTechPtr()->GetMacrosRef().back(); // last macro

  LayerRect tmpLayerRect;
  Rect2D<double> tmpRect;

  auto geometry = obs->geometries();
  int numItems = geometry->numItems();

  OBS *obs_ptr = last_macro.GetObs();

  //bool isNewLayerRect = true;
  LayerRect *layer_rect_ptr = nullptr;
  for (int i = 0; i < numItems; ++i) {
    if (geometry->itemType(i) == lefiGeomLayerE) {
      std::string layer_name(geometry->getLayer(i));
      layer_rect_ptr = obs_ptr->AddLayerRect(layer_name);
    } else if (geometry->itemType(i) == lefiGeomRectE) {
      double x1 = geometry->getRect(i)->xl;
      double y1 = geometry->getRect(i)->yl;
      double x2 = geometry->getRect(i)->xh;
      double y2 = geometry->getRect(i)->yh;
      PhyDBExpects(layer_rect_ptr != nullptr,
                   "unexpected error in getLefObs()");
      layer_rect_ptr->AddRect(
          std::min(x1, x2),
          std::min(y1, y2),
          std::max(x1, x2),
          std::max(y1, y2)
      );
    } else {
      std::cout << "Warning: unsupported OBS\n";
      continue;
    }
  }

  return 0;
}

int getLefCornerSpacing(PhyDB *phy_db_ptr, const std::string &stringProp) {
  std::istringstream istr(stringProp);
  std::string token;
  Layer &last_layer =
      phy_db_ptr->GetTechPtr()->GetLayersRef().back(); //write to the last one
  auto corner_spacing = last_layer.GetCornerSpacing();
  while (!istr.eof()) {
    istr >> token;
    if (token == "EXCEPTEOL") {
      double eol_width;
      istr >> eol_width;
      corner_spacing->SetEOLWidth(eol_width);
    } else if (token == "WIDTH") {
      double width;
      istr >> width;
      corner_spacing->AddWidth(width);
    } else if (token == "SPACING") {
      double spacing;
      istr >> spacing;
      corner_spacing->AddSpacing(spacing);
    }
  }
  return 0;
}

int getLefLayers(lefrCallbackType_e type, lefiLayer *layer, lefiUserData data) {
  if (type != lefrLayerCbkType) {
    std::cout << "Type is not lefrLayerCbkType!" << std::endl;
    exit(1);
  }

  auto *phy_db_ptr = (PhyDB *) data;

  if (strcmp(layer->type(), "ROUTING") == 0) {
    std::string metal_layer_name(layer->name());
    std::string layer_type(layer->type());
    std::string direction(layer->direction());

    Layer &last_layer = *(phy_db_ptr->AddLayer(
        metal_layer_name,
        StrToLayerType(layer_type),
        StrToMetalDirection(direction))
    );

    last_layer.SetWidth(layer->width());
    if (layer->hasMinwidth()) {
      last_layer.SetMinWidth(layer->minwidth());
    } else {
      last_layer.SetMinWidth(layer->width());
    }

    if (layer->hasXYPitch()) {
      last_layer.SetPitch(layer->pitchX(), layer->pitchY());
    } else {
      last_layer.SetPitch(layer->pitch(), layer->pitch());
    }
    last_layer.SetOffset(layer->offset());

    // read minArea rule
    if (layer->hasArea()) {
      last_layer.SetArea(layer->area());
    }

    if (layer->numProps() > 1) {
      std::cout << "ignore some unsupported properties for layer:"
                << layer->name() << std::endl;
    }
    for (int i = 0; i < layer->numProps(); i++) {
      if (std::string(layer->propName(i)) == "LEF58_CORNERSPACING"
          && layer->propIsString(i)) {
        getLefCornerSpacing(phy_db_ptr, layer->propValue(i));
      } else {
        std::cout << "WARNING: UNSUPPORTED PROPERTY: "
                  << layer->propName(i) << "\n";
      }
    }

    // read spacing rule
    for (int i = 0; i < layer->numSpacing(); ++i) {
      EolSpacing tmpSpacing;
      double spacing = layer->spacing(i);
      double eol_width = 0, eol_within = 0, par_edge = 0, par_within = 0;
      if (layer->hasSpacingEndOfLine(i)) {

        eol_width = layer->spacingEolWidth(i);
        eol_within = layer->spacingEolWithin(i);

        if (layer->hasSpacingParellelEdge(i)) {
          par_edge = layer->spacingParSpace(i);
          par_within = layer->spacingParWithin(i);
        }

        last_layer.AddEolSpacing(
            spacing,
            eol_width,
            eol_within,
            par_edge,
            par_within
        );

      } else {
        std::cout << "warning: no eol spacing!\n";
        last_layer.SetSpacing(spacing);
      }
    }

    // read spacingTable

    for (int i = 0; i < layer->numSpacingTable(); ++i) {
      auto spTable = layer->spacingTable(i);
      std::vector<double> v_parallel_run_length;
      std::vector<double> v_width;
      std::vector<double> v_spacing;
      if (spTable->isParallel() == 1) {
        auto parallel = spTable->parallel();

        for (int j = 0; j < parallel->numLength(); ++j) {
          v_parallel_run_length.push_back(parallel->length(j));
        }
        for (int j = 0; j < parallel->numWidth(); ++j) {
          v_width.push_back(parallel->width(j));
          for (int k = 0; k < parallel->numLength(); ++k) {
            v_spacing.push_back(parallel->widthSpacing(j, k));
          }
        }
        last_layer.SetSpacingTable(
            parallel->numLength(),
            parallel->numWidth(),
            v_parallel_run_length,
            v_width,
            v_spacing
        );

      } else if (spTable->isInfluence()) {
        auto influence = spTable->influence();
        for (int entry = 0; entry < influence->numInfluenceEntry();
             entry++) {
          last_layer.AddSpacingTableInfluence(
              influence->width(entry),
              influence->distance(entry),
              influence->spacing(entry));
        }
      } else {
        std::cout << "ignore unsupported spacing table!" << std::endl;
        //exit(1);
      }
    }

  } else if (strcmp(layer->type(), "CUT") == 0) { // cut layer
    std::string metal_layer_name(layer->name());
    std::string layer_type(layer->type());

    Layer &last_layer = *(phy_db_ptr->AddLayer(metal_layer_name,
                                               StrToLayerType(layer_type)));

    last_layer.SetWidth(layer->width());
    // read spacing constraint
    for (int i = 0; i < layer->numSpacing(); ++i) {

      if (layer->hasSpacingAdjacent(i)) {
        double spacing = layer->spacing(i);
        int adjacent_cuts = layer->spacingAdjacentCuts(i);
        int cut_within = layer->spacingAdjacentWithin(i);
        last_layer.SetAdjCutSpacing(spacing, adjacent_cuts, cut_within);
      } else {
        last_layer.SetSpacing(layer->spacing(i));
      }
    }

  } else {
    if (std::string(layer->name()) != "OVERLAP")
      std::cout << "unsupported layer type: " << layer->name() << ": "
                << layer->type() << std::endl;
  }

  return 0;
}

int getLefVias(lefrCallbackType_e type, lefiVia *via, lefiUserData data) {
  if (type != lefrViaCbkType) {
    std::cout << "Type is not lefrViaCbkType!" << std::endl;
    exit(1);
  }

  auto *phy_db_ptr = (PhyDB *) data;
  std::string via_name = via->name();
  LefVia &last_via = *(phy_db_ptr->AddLefVia(via_name));
  if (via->hasDefault())
    last_via.SetDefault();
  else
    last_via.UnsetDefault();

  if (via->numLayers() != 3) {
    std::cout << "Error: unsupported via (via layers != 3) " << via->name()
              << std::endl;
    exit(1);
  }
  std::string layer_name[3];
  std::vector<Rect2D<double>> rects[3];
  for (int i = 0; i < via->numLayers(); ++i) {
    layer_name[i] = via->layerName(i);
    for (int j = 0; j < via->numRects(i); ++j) {
      rects[i].emplace_back(
          via->xl(i, j),
          via->yl(i, j),
          via->xh(i, j),
          via->yh(i, j)
      );
    }
  }
  last_via.SetLayerRect(
      layer_name[0],
      rects[0],
      layer_name[1],
      rects[1],
      layer_name[2],
      rects[2]
  );

  return 0;
}

int getLefViaRuleGenerates(lefrCallbackType_e type,
                           lefiViaRule *viaRule,
                           lefiUserData data) {

  if (type != lefrViaRuleCbkType) {
    std::cout << "Type is not lefrViaRuleCbkType!" << std::endl;
    exit(1);
  }

  std::string name = viaRule->name();

  auto *phy_db_ptr = (PhyDB *) data;
  ViaRuleGenerate
      &last_viarule_generate = *(phy_db_ptr->AddViaRuleGenerate(name));

  if (viaRule->hasDefault())
    last_viarule_generate.SetDefault();
  else
    last_viarule_generate.UnsetDefault();

  if (viaRule->numLayers() != 3) {
    std::cout << "Error: unsupported via" << std::endl;
    exit(1);
  }
  ViaRuleGenerateLayer layer[3];

  for (int i = 0; i < viaRule->numLayers(); ++i) {
    auto viaRuleLayer = viaRule->layer(i);
    std::string layer_name = viaRuleLayer->name();
    layer[i].SetLayerName(layer_name);
    if (viaRuleLayer->hasEnclosure()) {
      layer[i].SetEnclosure(
          viaRuleLayer->enclosureOverhang1(),
          viaRuleLayer->enclosureOverhang2()
      );
    }

    if (viaRuleLayer->hasRect()) {
      layer[i].SetRect(
          viaRuleLayer->xl(),
          viaRuleLayer->yl(),
          viaRuleLayer->xh(),
          viaRuleLayer->xh()
      );
    }

    if (viaRuleLayer->hasSpacing()) {
      layer[i].SetSpacing(
          viaRuleLayer->spacingStepX(),
          viaRuleLayer->spacingStepY()
      );
    }
  }
  last_viarule_generate.SetLayers(layer[0], layer[1], layer[2]);

  return 0;
}

int getDefDesign(defrCallbackType_e type, const char *str, defiUserData data) {
  auto *phy_db_ptr = (PhyDB *) data;
  if (type == defrDesignStartCbkType) {
    std::string design_name(str);
    phy_db_ptr->SetDefName(design_name);
  }
  return 0;
}

int getDefRow(defrCallbackType_e type, defiRow *row, defiUserData data) {
  if ((type != defrRowCbkType)) {
    std::cout << "Type is not defrRowCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;

  std::string row_name(row->name());
  std::string site_name(row->macro());
  std::string site_orientation(row->orientStr());
  int origX = row->x();
  int origY = row->y();
  int numX = row->xNum();
  int numY = row->yNum();
  int stepX = row->xStep();
  int stepY = row->yStep();

  phy_db_ptr->AddRow(
      row_name,
      site_name,
      site_orientation,
      origX,
      origY,
      numX,
      numY,
      stepX,
      stepY
  );

  return 0;
}

int getDefString(defrCallbackType_e type, const char *str, defiUserData data) {
  auto *phy_db_ptr = (PhyDB *) data;
  if (type == defrDesignStartCbkType) {
    std::string design_name(str);
    phy_db_ptr->SetDefName(design_name);
  }
  return 0;
}

int getDefVoid(defrCallbackType_e type, void *variable, defiUserData data) {
  if ((type != defrDesignEndCbkType)) {
    std::cout << "Type is not defrDesignEndCbkType!" << std::endl;
    exit(1);
  }
  return 0;
}

void getMaxAndMin(int number_of_points, const int *series, int &min, int &max) {
  PhyDBExpects(number_of_points > 0, "number of points is no more than 0?");
  max = series[0];
  min = series[0];
  for (int i = 1; i < number_of_points; ++i) {
    max = std::max(max, series[i]);
    min = std::min(min, series[i]);
  }
}

int getDefDieArea(defrCallbackType_e type, defiBox *box, defiUserData data) {
  auto *phy_db_ptr = (PhyDB *) data;
  if ((type != defrDieAreaCbkType)) {
    std::cout << "Type is not defrDieAreaCbkType!" << std::endl;
    exit(1);
  }
  defiPoints polygon_points = box->getPoint();
  int num_points = polygon_points.numPoints;
  if (polygon_points.numPoints == 2 || polygon_points.numPoints == 4) {
    // if two points are defined, specifies two corners of the bounding rectangle for the design.
    // the edges of the polygon must be parallel to the x or y-axis (45-degree shapes are not
    // allowed), and the last point is connected to the first point.
    int lower_x, lower_y, upper_x, upper_y;
    getMaxAndMin(num_points, polygon_points.x, lower_x, upper_x);
    getMaxAndMin(num_points, polygon_points.y, lower_y, upper_y);
    phy_db_ptr->SetDieArea(lower_x, lower_y, upper_x, upper_y);
  } else {
    std::vector<Point2D<int>> rectilinear_polygon_die_area;
    rectilinear_polygon_die_area.reserve(num_points);
    for (int i = 0; i < num_points; ++i) {
      rectilinear_polygon_die_area.emplace_back(
          polygon_points.x[i],
          polygon_points.y[i]
      );
    }
    phy_db_ptr->SetRectilinearPolygonDieArea(rectilinear_polygon_die_area);
  }

  return 0;
}

int getDefUnits(defrCallbackType_e type, double number, defiUserData data) {
  if ((type != defrUnitsCbkType)) {
    std::cout << "Type is not defrUnitsCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  phy_db_ptr->SetUnitsDistanceMicrons(number);
  return 0;
}

int getDefTracks(defrCallbackType_e type, defiTrack *track, defiUserData data) {
  if ((type != defrTrackCbkType)) {
    std::cout << "Type is not defrTrackCbkType!" << std::endl;
    exit(1);
  }

  auto *phy_db_ptr = (PhyDB *) data;

  std::string dir_str(track->macro());
  XYDirection direction = StrToXYDirection(dir_str);
  int start = track->x();
  int num_tracks = track->xNum();
  int step = track->xStep();
  std::vector<std::string> layer_names;

  for (int i = 0; i < track->numLayers(); i++) {
    std::string layerName = track->layer(i);
    layer_names.push_back(layerName);
  }
  phy_db_ptr->AddTrack(direction, start, num_tracks, step, layer_names);

  return 0;
}

int getDefCountNumber(defrCallbackType_e type, int num, defiUserData data) {
  std::string name;
  auto *phy_db_ptr = (PhyDB *) data;
  switch (type) {
    case defrComponentStartCbkType : {
      name = "COMPONENTS";
      phy_db_ptr->SetComponentCount(num);
      break;
    }
    case defrStartPinsCbkType : {
      name = "PINS";
      phy_db_ptr->SetIoPinCount(num);
      break;
    }
    case defrNetStartCbkType : {
      name = "NETS";
      phy_db_ptr->SetNetCount(num);
      break;
    }
    default : {
      name = "BOGUS";
      PhyDBExpects(false, "Unsupported count types: BOGUS");
    }
  }
  return 0;
}

int getDefComponents(
    defrCallbackType_e type,
    defiComponent *comp,
    defiUserData data
) {
  if (type != defrComponentCbkType) {
    std::cout << "Type is not defrComponentCbkType!" << std::endl;
    exit(1);
  }

  std::string comp_name(comp->id());
  std::string macro_name(comp->name());
  int llx = comp->placementX();
  int lly = comp->placementY();

  PlaceStatus place_status = phydb::PlaceStatus::UNPLACED;
  if (comp->isPlaced()) {
    place_status = phydb::PlaceStatus::PLACED;
  } else if (comp->isFixed()) {
    place_status = phydb::PlaceStatus::FIXED;
  } else if (comp->isUnplaced()) {
    place_status = phydb::PlaceStatus::UNPLACED;
    llx = 0;
    lly = 0;
  } else if (comp->isCover()) {
    place_status = phydb::PlaceStatus::COVER;
  } else {
    llx = 0;
    lly = 0;
  }

  CompOrient orient = CompOrient::N;
  if (place_status != phydb::PlaceStatus::UNPLACED) {
    orient = StrToCompOrient(comp->placementOrientStr());
  }

  CompSource source = CompSource::NETLIST;
  if (comp->hasSource()) {
    source = StrToCompSource(comp->source());
  }

  auto *phy_db_ptr = (PhyDB *) data;
  Macro *macro_ptr = phy_db_ptr->GetMacroPtr(macro_name);
  PhyDBExpects(macro_ptr != nullptr,
               "Cannot find " + macro_name + " in PhyDB");
  phy_db_ptr->AddComponent(
      comp_name,
      macro_ptr,
      place_status,
      llx, lly,
      orient,
      source
  );

  return 0;
}

int getDefIOPins(defrCallbackType_e type, defiPin *pin, defiUserData data) {
  if (type != defrPinCbkType) {
    std::cout << "Type is not defrPinCbkType!" << std::endl;
    exit(1);
  }

  std::string iopin_name(pin->pinName());
  std::string signal_direction;
  if (pin->hasDirection())
    signal_direction = std::string(pin->direction());
  std::string signal_use = "SIGNAL";
  if (pin->hasUse()) {
    signal_use = std::string(pin->use());
  }

  auto *phy_db_ptr = (PhyDB *) data;
  IOPin *io_pin_ptr = phy_db_ptr->AddIoPin(
      iopin_name,
      StrToSignalDirection(signal_direction),
      StrToSignalUse(signal_use)
  );

  int iopin_x = 0;
  int iopin_y = 0;
  PlaceStatus place_status = phydb::PlaceStatus::UNPLACED;
  CompOrient orient = phydb::CompOrient::N;
  if (pin->isPlaced()) {
    place_status = phydb::PlaceStatus::PLACED;
    iopin_x = pin->placementX();
    iopin_y = pin->placementY();
    std::string str_orient(pin->orientStr());
    orient = StrToCompOrient(str_orient);
  } else if (pin->isUnplaced()) {
    place_status = phydb::PlaceStatus::UNPLACED;
  } else if (pin->isFixed()) {
    place_status = phydb::PlaceStatus::FIXED;
    iopin_x = pin->placementX();
    iopin_y = pin->placementY();
    std::string str_orient(pin->orientStr());
    orient = StrToCompOrient(str_orient);
  } else if (pin->isCover()) {
    place_status = phydb::PlaceStatus::COVER;
    iopin_x = pin->placementX();
    iopin_y = pin->placementY();
    std::string str_orient(pin->orientStr());
    orient = StrToCompOrient(str_orient);
  }
  io_pin_ptr->SetPlacement(place_status, iopin_x, iopin_y, orient);

  if (pin->hasPort()) {
    std::cout << "Error: multiple pin ports existing in DEF" << std::endl;
    exit(1);
  } else {
    for (int i = 0; i < pin->numLayer(); ++i) {
      int x1 = 0, y1 = 0, x2 = 0, y2 = 0;
      pin->bounds(i, &x1, &y1, &x2, &y2);
      std::string layer_name(pin->layer(i));
      io_pin_ptr->SetShape(
          layer_name,
          std::min(x1, x2),
          std::min(y1, y2),
          std::max(x1, x2),
          std::max(y1, y2)
      );
    }
  }

  return 0;

}

void addNetGeometry(defiNet *net, PhyDB *phy_db_ptr, bool specialnet) {
  #define BEN_DEBUG

  double ext_length = specialnet ? 0 : -1;

  
  #ifdef BEN_DEBUG
    std::cout << "START OF NET (" << net->name() << ")" << std::endl;
  #endif
  // iterate over the wires of the net, initialize net segment numberer

  int net_segment_id = 0;
  for (int i = 0; i < net->numWires(); i++) {
    defiWire *wire = net->wire(i);

    #ifdef BEN_DEBUG
      std::cout << "    START OF WIRE #" << i << std::endl;
    #endif

    // iterate over the paths of the wire
    for (int j = 0; j < wire->numPaths(); j++) {
      #ifdef BEN_DEBUG
        std::cout << "        START OF PATH #" << j << std::endl;
      #endif

      // initialize path traversal, centerline vector, layer tracker
      defiPath *path = wire->path(j);
      path->initTraverse();

      const char *curr_layer = nullptr;
      double curr_wire_width = -1;
      std::vector<Point2D<double>> centerline;

      // traverse path
      for (int t = path->next(); t != DEFIPATH_DONE; t = path->next()) {
        switch(t) {
          case DEFIPATH_LAYER: {

            // if there was a previous layer in the path register that geometry
            if (curr_layer != NULL) {
              phy_db_ptr->AddWireSegmentGeometryFromCenterline(centerline, curr_layer, net_segment_id, net->name(), ext_length, curr_wire_width);
              ext_length = specialnet ? 0 : -1; // reset ext_length
              curr_wire_width = -1; // reset wire width
            }
            curr_layer = path->getLayer();
            #ifdef BEN_DEBUG
              std::cout << "            LAYER: " << curr_layer << std::endl;
            #endif
            break;
          }
          case DEFIPATH_VIA: {
            const char *via_name = path->getVia();
            LefVia *lef_via_ptr = phy_db_ptr->GetLefViaPtr(via_name);
            DefVia *def_via_ptr = phy_db_ptr->GetDefViaPtr(via_name);
            if (centerline.size() != 1) {
              std::cout << "strange! expected one point in centerline prior to via..." << std::endl;
            }
            if (lef_via_ptr && def_via_ptr) {
              std::cout << "problem! this should never happen: via defined in both lef and def, exiting" << std::endl;
              exit(1);
            }
            else if (lef_via_ptr) {
              phy_db_ptr->AddLefViaGeometry(lef_via_ptr, net_segment_id, net->name(), centerline[0]);
            }
            else if (def_via_ptr) {
              phy_db_ptr->AddDefViaGeometry(def_via_ptr, net_segment_id, net->name(), centerline[0]);
            } else {
              std::cout << "problem! unrecognized via name: (" << via_name << ")" << std::endl;
              exit(1);
            }
            centerline.clear();

            #ifdef BEN_DEBUG 
              std::cout << "            VIA: " << via_name << std::endl;
            #endif
            break;
          }
          case DEFIPATH_WIDTH: {
            int w = path->getWidth();
            curr_wire_width = w;
            #ifdef BEN_DEBUG
              std::cout << "            WIDTH: " << w << std::endl;
            #endif
            break;
          }
          case DEFIPATH_POINT: {
            int x, y;
            path->getPoint(&x, &y);
            #ifdef BEN_DEBUG
              std::cout << "            POINT: (" << x << ", " << y << ")" << std::endl;
            #endif
            centerline.push_back(Point2D<double>(x, y));
            
            
            break;
          }
          case DEFIPATH_FLUSHPOINT:
            #ifdef BEN_DEBUG
              int x, y, e;
              path->getFlushPoint(&x, &y, &e);
              #ifdef BEN_DEBUG
                std::cout << "            FLUSHPOINT: (" << x << ", " << y << ", " << e << ")" << std::endl;
              #endif
              ext_length = e;
              centerline.push_back(Point2D<double>(x, y));
            #endif
            break;
          case DEFIPATH_TAPER:
            #ifdef BEN_DEBUG
              std::cout << "            taper" << std::endl;
            #endif
            break;
          case DEFIPATH_TAPERRULE:
            #ifdef BEN_DEBUG
              std::cout << "            taperrule" << std::endl;
            #endif
            break;
          case DEFIPATH_SHAPE:
            #ifdef BEN_DEBUG
              std::cout << "            shape " << path->getShape() << std::endl;
            #endif
            break;
          case DEFIPATH_STYLE:
            #ifdef BEN_DEBUG  
              std::cout << "            style" << std::endl;
            #endif
            break;
          case DEFIPATH_VIAROTATION:
            #ifdef BEN_DEBUG
              std::cout << "            viarotation" << std::endl;
            #endif
            break;
          case DEFIPATH_RECT: {
            int dx1, dy1, dx2, dy2;
            path->getViaRect(&dx1, &dy1, &dx2, &dy2);
            if (centerline.size() != 1) {
              std::cout << "strange! expected one point in centerline prior to rect..." << std::endl;
            }

            Rect2D<double> rect(centerline[0].x + dx1, centerline[0].y + dy1, centerline[0].x + dx2, centerline[0].y + dy2);
            phy_db_ptr->AddRectGeometry(curr_layer, net_segment_id, net->name(), rect);
            centerline.clear();
            #ifdef BEN_DEBUG
              std::cout << "            VIA RECT: (deltax1=" << dx1 << " deltay1=" << dy1 << " deltax2=" << dx2 << " deltay2=" << dy2 << ")" << std::endl;
            #endif
            break;
          }
          case DEFIPATH_VIADATA:
            break;
          case DEFIPATH_VIRTUALPOINT:
            break;
          case DEFIPATH_MASK:
            break;
          case DEFIPATH_VIAMASK:
            break;
          default: {
            std::cout << "unrecognized path element type: " << t << ", exiting" << std::endl;
          }
        }
      }
      // at end of path register the geometry of current segment, clear layer
      if (!centerline.empty()) phy_db_ptr->AddWireSegmentGeometryFromCenterline(centerline, curr_layer, net_segment_id, net->name(), ext_length, curr_wire_width);
      ext_length = specialnet ? 0 : -1;
      curr_wire_width = -1; // reseting wire width
      curr_layer = nullptr;

      #ifdef BEN_DEBUG
        std::cout << "        END OF PATH #" << j << std::endl;
      #endif
    }
    #ifdef BEN_DEBUG
      std::cout << "    END OF WIRE #" << i << std::endl;
    #endif
  }
  #ifdef BEN_DEBUG
    std::cout << "END OF NET" << std::endl;
  #endif
}

int getDefNets(defrCallbackType_e type, defiNet *net, defiUserData data) {
  if (type != defrNetCbkType) {
    std::cout << "Type is not defrNetCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;

  std::string net_name(net->name());
  phy_db_ptr->AddNet(net_name);

  for (int i = 0; i < net->numConnections(); i++) {
    std::string comp_name(net->instance(i));
    std::string pin_name(net->pin(i));
    if (comp_name == "PIN") {
      phy_db_ptr->AddIoPinToNet(pin_name, net_name);
    } else {
      phy_db_ptr->AddCompPinToNet(comp_name, pin_name, net_name);
    }
  }

  addNetGeometry(net, phy_db_ptr, false);

  return 0;
}

int getDefSNets(defrCallbackType_e type, defiNet *net, defiUserData data) {

  if (type != defrSNetCbkType) {
    std::cout << "Type is not defr(S)NetCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  std::string name = net->name();
  std::string use_str = net->use();
  SignalUse use = StrToSignalUse(use_str);

  auto *phydb_snet = phy_db_ptr->AddSNet(name, use);

  for (int i = 0; i < (int) net->numPolygons(); i++) {
    std::string layer_name = net->polygonName(i);
    auto points = net->getPolygon(i);
    auto phydb_polygon = phydb_snet->AddPolygon(layer_name);
    for (int j = 0; j < points.numPoints; j++) {
      phydb_polygon->AddRoutingPoint(points.x[j], points.y[j]);
    }
  }

  // read pre-route
  for (int i = 0; i < (int) net->numWires(); i++) {
    defiWire *tmpWire = net->wire(i);
    // each path is a def line
    for (int j = 0; j < (int) tmpWire->numPaths(); j++) {
      auto *phydb_path = phydb_snet->AddPath();
      defiPath *path = tmpWire->path(j);
      path->initTraverse();
      // initialize

      int pathId;
      //bool hasBeginPoint = false;
      while ((pathId = path->next()) != DEFIPATH_DONE) {
        //cout << "  pathId = " << pathId << endl;
        switch (pathId) {
          case DEFIPATH_LAYER: {
            auto layer_name = std::string(path->getLayer());
            phydb_path->SetLayerName(layer_name);
            break;
          }
          case DEFIPATH_VIA: {
            auto via_name = std::string(path->getVia());
            phydb_path->SetViaName(via_name);
            break;
          }
          case DEFIPATH_WIDTH: {
            phydb_path->SetWidth(path->getWidth());
            break;
          }
          case DEFIPATH_POINT: {
            int X, Y;
            path->getPoint(&X, &Y);
            phydb_path->AddRoutingPoint(X, Y);
            break;
          }
          case DEFIPATH_FLUSHPOINT: {
            int X, Y, ext;
            path->getFlushPoint(&X, &Y, &ext);
            phydb_path->AddRoutingPoint(X, Y, ext);
            break;
          }
          case DEFIPATH_SHAPE: {
            auto shape = std::string(path->getShape());
            phydb_path->SetShape(shape);
            break;
          }
          case DEFIPATH_RECT: {
            int x1, y1, x2, y2;
            path->getViaRect(&x1, &y1, &x2, &y2);
            phydb_path->SetRect(
                std::min(x1, x2),
                std::min(y1, y2),
                std::max(x1, x2),
                std::max(y1, y2)
            );
            break;
          }
          case DEFIPATH_VIRTUALPOINT: {
            int X, Y;
            path->getPoint(&X, &Y);
            phydb_path->AddRoutingPoint(X, Y);
            break;
          }
          default : {
            std::cout << " net " << net->name()
                      << " unknown pathId "
                      << pathId << "\n";
            break;
          }
        }
      }
    } // end_ path
  } // end_ wire

  addNetGeometry(net, phy_db_ptr, true);


  return 0;
}

int getDefVias(defrCallbackType_e type, defiVia *via, defiUserData data) {
  if ((type != defrViaCbkType)) {
    std::cout << "Type is not defrViaCbkType!" << std::endl;
    exit(1);
  }

  auto *phy_db_ptr = (PhyDB *) data;
  std::string via_name = via->name();
  DefVia &last_via = *(phy_db_ptr->AddDefVia(via_name));
  // viaRule defined via
  if (via->hasViaRule()) {
    char *via_rule_name_;
    char *botLayer;
    char *cutLayer;
    char *topLayer;
    int xSize, ySize, xCutSpacing, yCutSpacing, xBotEnc, yBotEnc, xTopEnc,
        yTopEnc;

    via->viaRule(
        &via_rule_name_,
        &xSize,
        &ySize,
        &botLayer,
        &cutLayer,
        &topLayer,
        &xCutSpacing,
        &yCutSpacing,
        &xBotEnc,
        &yBotEnc,
        &xTopEnc,
        &yTopEnc
    );
    last_via.via_rule_name_ = via_rule_name_;
    last_via.cut_size_.Set(xSize, ySize);
    last_via.layers_[0] = std::string(botLayer);
    last_via.layers_[1] = std::string(cutLayer);
    last_via.layers_[2] = std::string(topLayer);

    last_via.cut_spacing_.Set(xCutSpacing, yCutSpacing);

    last_via.bot_enc_.Set(xBotEnc, yBotEnc);
    last_via.top_enc_.Set(xTopEnc, yTopEnc);

    int xOrigin = 0;
    int yOrigin = 0;
    if (via->hasOrigin()) {
      via->origin(&xOrigin, &yOrigin);
    }
    last_via.origin_.Set(xOrigin, yOrigin);

    int xBotOffset = 0;
    int yBotOffset = 0;
    int xTopOffset = 0;
    int yTopOffset = 0;
    if (via->hasOffset()) {
      via->offset(&xBotOffset, &yBotOffset, &xTopOffset, &yTopOffset);
    }
    last_via.bot_offset_.Set(xBotOffset, yBotOffset);
    last_via.top_offset_.Set(xTopOffset, yTopOffset);

    int num_cut_rows_ = 1;
    int num_cut_cols_ = 1;
    if (via->hasRowCol()) {
      via->rowCol(&num_cut_rows_, &num_cut_cols_);
    }
    last_via.num_cut_rows_ = num_cut_rows_;
    last_via.num_cut_cols_ = num_cut_cols_;

  } else // RECT defined via
  {
    if (via->numPolygons()) {
      std::cout << "Error: unsupport polygon in def via" << std::endl;
      exit(1);
    }
    char *layer_name_;
    int xl;
    int yl;
    int xh;
    int yh;

    for (int i = 0; i < via->numLayers(); ++i) {
      via->layer(i, &layer_name_, &xl, &yl, &xh, &yh);
      Rect2DLayer<int> tmpRect2DLayer;
      std::string layer_name(layer_name_);
      tmpRect2DLayer.Set(layer_name, xl, yl, xh, yh);
      last_via.rect2d_layers.push_back(tmpRect2DLayer);
    }
    //TODO:
  }

  return 0;
}

int getDefGcellGrid(
    defrCallbackType_e type,
    defiGcellGrid *gcellGrid,
    defiUserData data
) {
  if ((type != defrGcellGridCbkType)) {
    std::cout << "Type is not defrGcellGridCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  auto dir_str = std::string(gcellGrid->macro());
  XYDirection direction = StrToXYDirection(dir_str);
  phy_db_ptr->AddGcellGrid(
      direction,
      gcellGrid->x(),
      gcellGrid->xNum(),
      gcellGrid->xStep()
  );
  return 0;
}

int getDefVersion(defrCallbackType_e type, double version, defiUserData data) {
  if ((type != defrVersionCbkType)) {
    std::cout << "Type is not defrVersionCbkType!" << std::endl;
    exit(1);
  }
  ((PhyDB *) data)->SetDefVersion(version);
  return 0;
}

int getDefBlockageStart(
    defrCallbackType_e type,
    int num,
    defiUserData data
) {
  PhyDBExpects(type == defrBlockageStartCbkType,
               "Type is not defrBlockageStartCbkType!");
  auto *phy_db_ptr = (PhyDB *) data;
  PhyDBExpects(num >= 0, "Negative number of blockages?");
  phy_db_ptr->SetBlockageCount(num);
  return 0;
}

int getDefBlockage(
    defrCallbackType_e type,
    defiBlockage *defi_blockage,
    defiUserData data
) {
  PhyDBExpects(type == defrBlockageCbkType, "Type is not defrBlockageCbkType!");
  auto *phydb_ptr = (PhyDB *) data;
  Blockage *blockage = phydb_ptr->AddBlockage();

  if (defi_blockage->hasLayer()) {
    std::string layer_name(defi_blockage->layerName());
    Layer *layer_ptr = phydb_ptr->GetLayerPtr(layer_name);
    PhyDBExpects(layer_ptr != nullptr, "Cannot find layer: " + layer_name);
    blockage->SetLayer(layer_ptr);

    if (defi_blockage->hasSlots()) {
      blockage->SetSlots();
    }
    if (defi_blockage->hasFills()) {
      blockage->SetFills();
    }
    if (defi_blockage->hasPushdown()) {
      blockage->SetPushdown();
    }
    if (defi_blockage->hasExceptpgnet()) {
      blockage->SetExceptpgnet();
    }
    if (defi_blockage->hasComponent()) {
      std::string comp_name(defi_blockage->layerComponentName());
      Component *comp_ptr = phydb_ptr->GetComponentPtr(comp_name);
      PhyDBExpects(comp_ptr != nullptr, "Cannot find component: " + comp_name);
      blockage->SetComponent(comp_ptr);
    }
    if (defi_blockage->hasSpacing()) {
      blockage->SetSpacing(defi_blockage->minSpacing());
    }
    if (defi_blockage->hasDesignRuleWidth()) {
      blockage->SetDesignRuleWidth(defi_blockage->designRuleWidth());
    }
    if (defi_blockage->hasMask()) {
      blockage->SetMaskNum(defi_blockage->mask());
    }
  } else if (defi_blockage->hasPlacement()) {
    blockage->SetPlacement();
    if (defi_blockage->hasSoft()) {
      blockage->SetSoft();
    }
    if (defi_blockage->hasPartial()) {
      blockage->SetPartial(defi_blockage->placementMaxDensity());
    }
    if (defi_blockage->hasPushdown()) {
      blockage->SetPushdown();
    }
    if (defi_blockage->hasComponent()) {
      std::string comp_name(defi_blockage->placementComponentName());
      Component *comp_ptr = phydb_ptr->GetComponentPtr(comp_name);
      PhyDBExpects(comp_ptr != nullptr, "Cannot find component: " + comp_name);
      blockage->SetComponent(comp_ptr);
    }
  } else {
    PhyDBExpects(false, "This blockage has no layer and placement?");
  }

  int num_rects = defi_blockage->numRectangles();
  for (int i = 0; i < num_rects; ++i) {
    int lx = defi_blockage->xl(i);
    int ly = defi_blockage->yl(i);
    int ux = defi_blockage->xh(i);
    int uy = defi_blockage->yh(i);
    blockage->AddRect(lx, ly, ux, uy);
  }
  int num_polygons = defi_blockage->numPolygons();
  for (int i = 0; i < num_polygons; ++i) {
    auto defi_polygon = defi_blockage->getPolygon(i);
    auto &polygon = blockage->AddPolygon();;
    int num_points = defi_polygon.numPoints;
    for (int j = 0; j < num_points; ++j) {
      polygon.AddPoint(defi_polygon.x[j], defi_polygon.y[j]);
    }
  }

  return 0;
}

int getDefBusBit(
    defrCallbackType_e type,
    const char *BusBit,
    defiUserData data
) {
  if ((type != defrBusBitCbkType)) {
    std::cout << "Type is not defrBusBitCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  std::string bus_bit_char(BusBit);
  phy_db_ptr->SetDefBusBitChar(bus_bit_char);
  return 0;
}

int getDefDivider(
    defrCallbackType_e type,
    const char *divider,
    defiUserData data
) {
  if ((type != defrDividerCbkType)) {
    std::cout << "Type is not defrDividerCbkType!" << std::endl;
    exit(1);
  }
  auto *phy_db_ptr = (PhyDB *) data;
  std::string divider_chars(divider);
  phy_db_ptr->SetDefDividerChar(divider_chars);
  return 0;
}


void Si2ReadLef(PhyDB *phy_db_ptr, std::string const &lef_file_name) {
  FILE *f;
  int res;

  lefrInitSession(1);

  lefrSetUserData((lefiUserData) phy_db_ptr);

  lefrSetVersionCbk(getLefVersion);
  lefrSetMacroBeginCbk(getLefMacrosBegin);
  lefrSetMacroCbk(getLefMacros);
  lefrSetMacroEndCbk(getLefMacrosEnd);
  lefrSetUnitsCbk(getLefUnits);
  lefrSetManufacturingCbk(getLefManufacturingGrid);
  lefrSetSiteCbk(getLefSite);
  lefrSetPinCbk(getLefPins);
  lefrSetObstructionCbk(getLefObs);
  lefrSetLayerCbk(getLefLayers);
  lefrSetViaCbk(getLefVias);
  lefrSetViaRuleCbk(getLefViaRuleGenerates);

  if ((f = fopen(lef_file_name.c_str(), "r")) == nullptr) {
    std::cout << "Couldn't open lef file" << std::endl;
    exit(2);
  }

  res = lefrRead(f, lef_file_name.c_str(), (lefiUserData) phy_db_ptr);
  if (res != 0) {
    std::cout << "LEF parser returns an error!" << std::endl;
    exit(2);
  }
  fclose(f);

  lefrClear();
}

void Si2ReadDef(PhyDB *phy_db_ptr, std::string const &def_file_name) {
  FILE *f;
  int res;

  defrInit();
  defrReset();

  defrInitSession(1);

  defrSetUserData((defiUserData) phy_db_ptr);

  defrSetVersionCbk(getDefVersion);
  defrSetBusBitCbk(getDefBusBit);
  defrSetDividerCbk(getDefDivider);

  defrSetDesignCbk(getDefString);
  defrSetDesignEndCbk(getDefVoid);
  defrSetDieAreaCbk(getDefDieArea);
  defrSetUnitsCbk(getDefUnits);
  defrSetRowCbk(getDefRow);
  defrSetTrackCbk(getDefTracks);

  defrSetComponentStartCbk(getDefCountNumber);
  defrSetComponentCbk(getDefComponents);

  defrSetStartPinsCbk(getDefCountNumber);
  defrSetPinCbk(getDefIOPins);

  defrSetBlockageStartCbk(getDefBlockageStart);
  defrSetBlockageCbk(getDefBlockage);

  defrSetNetStartCbk(getDefCountNumber);
  defrSetSNetCbk(getDefSNets);
  defrSetAddPathToNet();
  defrSetNetCbk(getDefNets);

  defrSetViaCbk(getDefVias);
  defrSetGcellGridCbk(getDefGcellGrid);


  if ((f = fopen(def_file_name.c_str(), "r")) == 0) {
    std::cout << "Couldn't open def file" << std::endl;
    exit(2);
  }
  res = defrRead(f, def_file_name.c_str(), (defiUserData) phy_db_ptr, 1);
  if (res != 0) {
    std::cout << "DEF parser returns an error!" << std::endl;
    exit(2);
  }
  fclose(f);

  defrClear();
}

/**
 * Check the unit in the DEF file, and check if they have the same value.
 * In general, they can be different, but in our flow, they are supposed to be the same.
 *
 * @param type, callback type.
 * @param number, UNITS DISTANCE MICRONS in DEF.
 * @param data, void pointer to PhyDB.
 * @return 0 for success.
 */
int CheckDefUnits(defrCallbackType_e type, double number, defiUserData data) {
  if ((type != defrUnitsCbkType)) {
    std::cout << "Type is not defrUnitsCbkType!" << std::endl;
    exit(1);
  }

  auto *phy_db_ptr = (PhyDB *) data;

  int cur_unit = (int) number;
  int existing_unit = phy_db_ptr->GetDesignPtr()->GetUnitsDistanceMicrons();
  PhyDBExpects(
      cur_unit == existing_unit,
      "UNITS DISTANCE MICRONS is not supposed to be changed in the placed DEF file"
  );

  return 0;
}

/**
 * For a certain component, check if this component exists in PhyDB.
 * If yes, override its placement info by the new one.
 *
 * @param type, callback type.
 * @param comp, pointer to the component created by the parser.
 * @param data, void pointer to PhyDB.
 * @return 0 for success.
 */
int LoadDefComponentLoc(defrCallbackType_e type,
                        defiComponent *comp,
                        defiUserData data) {
  if ((type != defrComponentCbkType)) {
    std::cout << "Type is not defrComponentCbkType!" << std::endl;
    exit(1);
  }

  std::string comp_name(comp->id());
  std::string macro_name(comp->name());
  int llx = comp->placementX();
  int lly = comp->placementY();

  PlaceStatus place_status = phydb::PlaceStatus::UNPLACED;
  if (comp->isPlaced()) {
    place_status = phydb::PlaceStatus::PLACED;
  } else if (comp->isFixed()) {
    place_status = phydb::PlaceStatus::FIXED;
  } else if (comp->isUnplaced()) {
    place_status = phydb::PlaceStatus::UNPLACED;
    llx = 0;
    lly = 0;
  } else if (comp->isCover()) {
    place_status = phydb::PlaceStatus::COVER;
  } else {
    llx = 0;
    lly = 0;
  }

  std::string orient(comp->placementOrientStr());

  auto *phy_db_ptr = (PhyDB *) data;
  PhyDBExpects(phy_db_ptr->IsComponentExisting(comp_name),
               "Component " + comp_name + " is not in PhyDB database");
  Component *comp_ptr = phy_db_ptr->GetComponentPtr(comp_name);
  comp_ptr->SetLocation(llx, lly);
  comp_ptr->SetOrientation(StrToCompOrient(orient));
  comp_ptr->SetPlacementStatus(place_status);

  return 0;
}

/**
 * Load a DEF file with placed components, and then override component locations in PhyDB.
 * This is useful for detailed placement and legalization of (multi-height) standard cell designs,
 * because Dali does not support detailed placement and legalization for multi-height standard cells.
 *
 * @param phy_db_ptr, the pointer to the PhyDB database.
 * @param def_file_name, the DEF file name which contains new component locations.
 * @return void.
 */
void Si2LoadPlacedDef(PhyDB *phy_db_ptr, std::string const &def_file_name) {
  FILE *f;
  int res;

  defrInit();
  defrReset();

  defrInitSession(1);

  defrSetUserData((defiUserData) phy_db_ptr);

  defrSetUnitsCbk(CheckDefUnits);

  defrSetComponentStartCbk(getDefCountNumber);
  defrSetComponentCbk(LoadDefComponentLoc);

  if ((f = fopen(def_file_name.c_str(), "r")) == nullptr) {
    std::cout << "Couldn't open def file" << std::endl;
    exit(2);
  }

  res = defrRead(f, def_file_name.c_str(), (defiUserData) phy_db_ptr, 1);
  if (res != 0) {
    std::cout << "DEF parser returns an error!" << std::endl;
    exit(2);
  }
  fclose(f);

  defrClear();
}

}



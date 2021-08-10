#include <time.h>

#include "defwriter.h"

namespace phydb {

int WriteVersion(defwCallbackType_e c, defiUserData ud) {
    int status;
    double version = ((PhyDB*) ud)->GetDefVersion();

    int int_version = version * 10;
    status = defwVersion(int_version / 10, int_version % 10);
    defwNewLine();
    return 0;
}

int WriteBusBit(defwCallbackType_e c, defiUserData ud) {
    int status;
    status = defwBusBitChars(((PhyDB*) ud)->GetDefBusBitChar().c_str());
    defwNewLine();
    return 0;
}

int WriteDivider(defwCallbackType_e c, defiUserData ud) {
    int status;
    status = defwDividerChar(((PhyDB*) ud)->GetDefDividerChar().c_str());
    defwNewLine();
    return 0;
}

int WriteDesignName(defwCallbackType_e c, defiUserData ud) {
  int status;

  status = defwDesignName(((PhyDB*) ud)->GetDefName().c_str());
  defwNewLine();
  return 0;
}

int WriteDesignEnd(defwCallbackType_e c, defiUserData ud) {
    defwNewLine();
    defwEnd();
    return 0;
}

int WriteUnitsDistanceMicrons(defwCallbackType_e c, defiUserData ud) {
    int status;

    status = defwUnits(((PhyDB*) ud)->GetDesignPtr()->GetUnitsDistanceMicrons());
    defwNewLine();
    return 0;
}

int WriteDieArea(defwCallbackType_e c, defiUserData ud) {
    int status;
    Rect2D<int> dieArea = ((PhyDB*) ud)->GetDieArea(); 
    status = defwDieArea(dieArea.LLX(), dieArea.LLY(), dieArea.URX(), dieArea.URY());
    defwNewLine();
    return 0;
}

int WriteRows(defwCallbackType_e type, defiUserData data) {
    int status;
    auto rows = ((PhyDB*) data)->GetRowVec();
    for(auto row : rows) {
        status = defwRowStr(row.name_.c_str(), row.site_name_.c_str(), row.orig_x_, row.orig_y_,
            row.site_orient_.c_str(), row.num_x_, row.num_y_, row.step_x_, row.step_y_); 
    }

    return 0;
}

int WriteTracks(defwCallbackType_e type, defiUserData data) {
    auto tracks = ((PhyDB*) data)->GetTracksRef();
    for(auto track : tracks) {
        int nlayers = track.GetLayerNames().size();
        const char** layer_names = new const char* [nlayers];
        for(int i = 0; i < nlayers; i++)
            layer_names[i] = track.GetLayerNames()[i].c_str();
        defwTracks(XYDirectionToStr(track.GetDirection()).c_str(), track.GetStart(), track.GetNTracks(), track.GetStep(), nlayers, layer_names); 
        
    }
    defwNewLine();
    return 0;
}

int WriteComponents(defwCallbackType_e type, defiUserData data) {
    auto components = ((PhyDB*) data)->GetDesignPtr()->GetComponentsRef();
    defwStartComponents(components.size());
    
    for(auto comp : components) {
        defwComponentStr(comp.GetName().c_str(),
                    comp.GetMacro()->GetName().c_str(),
                    0,
                    NULL, NULL, NULL, NULL, NULL,
                    0,
                    NULL, NULL, NULL, NULL,
                    PlaceStatusStr(comp.GetPlacementStatus()).c_str(),
                    comp.GetLocation().x, comp.GetLocation().y,
                    CompOrientStr(comp.GetOrientation()).c_str(),
                    0,
                    NULL,
                    0, 0, 
                    0, 0);
    }
    defwEndComponents();
    return 0;
}

int WriteIOPins(defwCallbackType_e type, defiUserData data) {
    auto iopins = ((PhyDB*) data)->GetDesignPtr()->GetIoPinsRef();
    defwStartPins(iopins.size());
    
    for(auto pin : iopins) {
        defwPinStr(pin.GetName().c_str(),
                pin.GetNetName().c_str(),
                0,
                SignalDirectionStr(pin.GetDirection()).c_str(),
                SignalUseStr(pin.GetUse()).c_str(),
                PlaceStatusStr(pin.GetPlacementStatus()).c_str(),
                pin.GetLocation().x,
                pin.GetLocation().y,
                CompOrientStr(pin.GetOrientation()).c_str(),
                pin.GetLayerName().c_str(),
                pin.GetRect().LLX(),
                pin.GetRect().LLY(),
                pin.GetRect().URX(),
                pin.GetRect().URY()
               );
    }
    defwEndPins();

    return 0;

}

int WriteNets(defwCallbackType_e type, defiUserData data) {
    auto nets = ((PhyDB*) data)->GetDesignPtr()->GetNetsRef();
    defwStartNets(nets.size());
    char* pin_str = "PIN";
    for(auto net : nets) {
        defwNet(net.GetName().c_str());
        auto component_names = net.GetComponentNamesRef();
        auto pin_names = net.GetPinNamesRef();
        auto iopin_names = net.GetIoPinNamesRef();
        for(int i = 0; i < iopin_names.size(); i++) {
            defwNetConnection(pin_str, iopin_names[i].c_str(), 0);
        }

        for(int i = 0; i < component_names.size(); i++) {
            defwNetConnection(component_names[i].c_str(), pin_names[i].c_str(), 0);
        }
       defwNetEndOneNet();
    }

    defwEndNets();

    return 0;
}

int WriteSNets(defwCallbackType_e c, defiUserData ud) {
   
    double coorX[3], coorY[3];
    auto snet_vec = ((PhyDB*) ud)->GetSNetRef();

    if(snet_vec.size())
        defwStartSpecialNets(snet_vec.size()); //Number of special nets
    else
        return 0;
    
    for(int snet_id = 0; snet_id < snet_vec.size(); snet_id++) {
        auto snet = snet_vec[snet_id];
        defwSpecialNet(snet.GetName().c_str());
        defwSpecialNetConnection("*", snet.GetName().c_str(), 0);
        defwSpecialNetUse(SignalUseStr(snet.GetUse()).c_str());
        
        auto paths = snet.GetPathRef();
        for(int i = 0; i < paths.size(); i++) {
            auto path = paths[i];

            if(i == 0)
                defwSpecialNetPathStart("ROUTED");
            else     
                defwSpecialNetPathStart("NEW");

            defwSpecialNetPathLayer(path.GetLayerName().c_str());
            defwSpecialNetPathWidth(path.GetWidth());
            defwSpecialNetPathShape("STRIPE");
            int num_path_point = (path.HasEndPoint())? 2 : 1;
            coorX[0] = path.GetBegin().x;
            coorY[0] = path.GetBegin().y;
            if(num_path_point == 2) {
                coorX[1] = path.GetEnd().x;
                coorY[1] = path.GetEnd().y;
            }
            else {
                coorX[1] = -1;
                coorY[1] = -1;
            }
            defwSpecialNetPathPoint(num_path_point, coorX, coorY);
            if(num_path_point == 1)
                defwSpecialNetPathVia(path.GetViaName().c_str());
        }
        defwSpecialNetPathEnd();
        defwSpecialNetEndOneNet();
    }
    
    defwEndSpecialNets();


    return 0;
}

std::string GetCurrentDateTime() {
    time_t     now = time(0);
    struct tm  tstruct;
    char       buf[80];
    tstruct = *localtime(&now);
    strftime(buf, sizeof(buf), "%Y-%m-%d.%X", &tstruct);

    return buf;
}


void Si2WriteDef(PhyDB *phy_db_ptr, string const &defFileName) {
    FILE* f;
    int res;

    if ((f = fopen(defFileName.c_str(),"w")) == 0) {
        cout <<"Couldn't open Write def file" <<endl;
        exit(2);
    }
    std::cout << "Writing def to " << defFileName << std::endl;
    int status = defwInitCbk(f);

    defwSetVersionCbk (WriteVersion);
    defwSetBusBitCbk (WriteBusBit);
    defwSetDividerCbk (WriteDivider);

    defwSetDesignCbk (WriteDesignName);
    defwSetDesignEndCbk (WriteDesignEnd);

    defwSetUnitsCbk (WriteUnitsDistanceMicrons);
    defwSetDieAreaCbk (WriteDieArea);
    defwSetRowCbk (WriteRows);

    defwSetTrackCbk (WriteTracks);
    defwSetComponentCbk (WriteComponents);
    defwSetPinCbk (WriteIOPins);
    defwSetNetCbk (WriteNets);
    defwSetSNetCbk (WriteSNets);


    fprintf(f, "###########################\n");
    fprintf(f, "# Written by PhyDB at ");
    fprintf(f, GetCurrentDateTime().c_str());
    fprintf(f, "\n");
    fprintf(f, "###########################\n");
    
    res = defwWrite(f, defFileName.c_str(), (defiUserData) phy_db_ptr);
    if (res != 0) {
        cout << "DEF Writer returns an error!" << endl;
        exit(2);
    }
    fclose(f);
    std::cout << "def writing completes" << std::endl;
}

}

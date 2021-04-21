#ifndef SI2WRITER_H_
#define SI2WRITER_H_

#include "phydb.h"
#include <lefrReader.hpp>
#include <defrReader.hpp>
#include <defwWriter.hpp>
#include <defwWriterCalls.hpp>
#include <lefwWriter.hpp>
#include <lefwWriterCalls.hpp>

namespace phydb {

int writeLefSite(lefwCallbackType_e c, lefiUserData ud);
int writeLefMacrosBegin(lefwCallbackType_e c, lefiUserData ud);
int writeLefMacros(lefwCallbackType_e c, lefiUserData ud);
int writeLefMacrosEnd(lefwCallbackType_e c, lefiUserData ud);
int writeLefString(lefwCallbackType_e c, lefiUserData ud);
int writeLefUnits(lefwCallbackType_e c, lefiUserData ud);
int writeLefManufacturingGrid(lefwCallbackType_e c, lefiUserData ud);
int writeLefPins(lefwCallbackType_e c, lefiUserData ud);
int writeLefObs(lefwCallbackType_e c, lefiUserData ud);
int writeLefLayers(lefwCallbackType_e c, lefiUserData ud);
int writeLefVias(lefwCallbackType_e c, lefiUserData ud);
int writeLefViaRuleGenerates(lefwCallbackType_e c, lefiUserData ud);

int writeDefVersion(defwCallbackType_e c, defiUserData ud);
int writeDefBusBit(defwCallbackType_e c, defiUserData ud);
int writeDefDivider(defwCallbackType_e c, defiUserData ud);
int writeDefString(defwCallbackType_e c, defiUserData ud);
int writeDefVoid(defwCallbackType_e c, defiUserData ud);
int writeDefDieArea(defwCallbackType_e c, defiUserData ud);
int writeDefUnits(defwCallbackType_e c, defiUserData ud);
int writeDefTracks(defwCallbackType_e c, defiUserData ud);
int writeDefComponents(defwCallbackType_e c, defiUserData ud);
int writeDefIOPins(defwCallbackType_e c, defiUserData ud);
int writeDefNets(defwCallbackType_e c, defiUserData ud);
int writeDefSNets(defwCallbackType_e c, defiUserData ud);
int writeDefVias(defwCallbackType_e c, defiUserData ud);
int writeDefGcellGrid(defwCallbackType_e c, defiUserData ud);
int writeDefRow(defwCallbackType_e c, defiUserData ud);

void Si2WriteLef(PhyDB *phy_db_ptr, string const &lefFileName);
void Si2WriteDef(PhyDB *phy_db_ptr, string const &defFileName);
void WriteCluter(PhyDB *phy_db_ptr, string const &clusterFileName);

}


#endif //SI2WRITER_H_
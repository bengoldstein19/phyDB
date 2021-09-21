 %skeleton "lalr1.cc" /* -*- C++ -*- */
 %require "3.0"
 %defines
 %define api.parser.class { Parser }

 %define api.token.constructor
 %define api.value.type variant
 %define parse.assert
 %define api.namespace { phydb }
 %code requires
 {
    #include <iostream>
    #include <string>
    #include <vector>
    #include <stdint.h>
    #include <string.h>
    #include "techconfig.h"

    using namespace std;

    namespace phydb {
        class Scanner;
        class Interpreter;
    }
}

%code top
{
    #include <iostream>
    #include "scanner.h"
    #include "parser.hpp"
    #include "interpreter.h"
    #include "location.hh"

    static phydb::Parser::symbol_type yylex(phydb::Scanner &scanner, phydb::Interpreter &driver) {
        return scanner.get_next_token();
    }
    
    using namespace phydb;
}

%lex-param { phydb::Scanner &scanner }
%lex-param { phydb::Interpreter &driver }
%parse-param { phydb::Scanner &scanner }
%parse-param { phydb::Interpreter &driver }
%locations
%define parse.trace
%define parse.error verbose

%define api.token.prefix {TOKEN_}

%token <std::string> EXTRACTION
%token <std::string> DIAGMODEL
%token <std::string> ON
%token <std::string> OFF
%token <std::string> LAYERCOUNT
%token <std::string> DENSITYRATE
%token <std::string> DENSITYMODEL
%token <std::string> DIST
%token <std::string> COUNT
%token <std::string> WIDTH
%token <std::string> WIDTH_TABLE
%token <std::string> ENTRIES
%token <double> NUMBER
%token <std::string> METAL
%token <std::string> RESOVER
%token <std::string> OVER
%token <std::string> UNDER
%token <std::string> DIAGUNDER
%token <std::string> OVERUNDER
%token <std::string> TABLE_END
%token <std::string> DENSITYMODEL_END

%token END 0

%type<std::string> extraction diagmodel layercount densityrate densitymodel info 
%type<std::string> relative_pos

%start tech_config_file

%%

tech_config_file : header model
;

header: info
    | header info
;

info: extraction 
    | diagmodel 
    | layercount 
    | densityrate 
;

extraction: EXTRACTION
;

diagmodel: DIAGMODEL ON
    {
        tech_config->SetDiagmodelOn(true);
    }
    | DIAGMODEL OFF
    {
        tech_config->SetDiagmodelOn(false);
    }
;

layercount: LAYERCOUNT NUMBER
    {
        tech_config->SetLayerCount(int($2));
    }
;

densityrate: DENSITYRATE NUMBER
    {
        tech_config->SetModelCount(int($2));
    }
    | densityrate NUMBER
    {
        tech_config->AddDataRate($2);
    }
;

model: densitymodel metal_tables end_densitymodel
;

densitymodel: DENSITYMODEL NUMBER
{
    tech_config->AddModel(int($2));
}
;

metal_tables: metal_header table
    | metal_tables table
    | metal_tables metal_header
;


metal_header: METAL NUMBER relative_pos WIDTH_TABLE NUMBER ENTRIES 
    | METAL NUMBER relative_pos WIDTH_TABLE NUMBER ENTRIES NUMBER
;

relative_pos: RESOVER 
    | OVER 
    | UNDER 
    | DIAGUNDER 
    | OVERUNDER
;

table: table_header table_start table_body table_end
    {
        auto model = tech_config->GetLastModel();
        if (model != nullptr) {
            model->MarkNothing();
        }
    }
;

table_header: simple_table_header
    | simple_table_header under_layer
;

simple_table_header: METAL NUMBER relative_pos NUMBER
    {
        auto model = tech_config->GetLastModel();
        if (model != nullptr) {
            int layer_index = (int) $4;
            if ($3 == "RESOVER") {
                model->res_over_.emplace_back(layer_index);
                model->MarkResOver();
            } else if ($3 == "OVER") {
                model->cap_over_.emplace_back(layer_index);
                model->MarkCapOver();
            } else if ($3 == "UNDER") {
                model->cap_under_.emplace_back(layer_index);
                model->MarkCapUnder();
            } else if ($3 == "DIAGUNDER") {
                model->cap_diagunder_.emplace_back(layer_index);
                model->MarkCapDiagUnder();
            } else {
                cout << "impossible\n";
                exit(1);
            }
        }
    }
;

under_layer: UNDER NUMBER
{
    auto model = tech_config->GetLastModel();
    if (model != nullptr) {
        int over_index = model->cap_over_.back().layer_index_;
        model->cap_over_.pop_back();

        int under_index = (int) $2;
        model->cap_overunder_.emplace_back(over_index, under_index);
        model->MarkCapOverUnder();
    }
}
;

table_start: DIST COUNT NUMBER WIDTH NUMBER
{
    auto model = tech_config->GetLastModel();
    int sz = (int) $3;
    double width = $5;
    if (model != nullptr) {
        if (model->tmp_res_over_ != nullptr) {
            model->tmp_res_over_->table_.reserve(sz);
            model->tmp_res_over_->width_ = width;
        }
        if (model->tmp_cap_over_ != nullptr) {
            model->tmp_cap_over_->table_.reserve(sz);
            model->tmp_cap_over_->width_ = width;
        }
        if (model->tmp_cap_under_ != nullptr) {
            model->tmp_cap_under_->table_.reserve(sz);
            model->tmp_cap_under_->width_ = width;
        }
        if (model->tmp_cap_diagunder_ != nullptr) {
            model->tmp_cap_diagunder_->table_.reserve(sz);
            model->tmp_cap_diagunder_->width_ = width;
        }
        if (model->tmp_cap_overunder_ != nullptr) {
            model->tmp_cap_overunder_->table_.reserve(sz);
            model->tmp_cap_overunder_->width_ = width;
        }
    }
}
;

table_body: 
    | table_body table_entry
;

table_entry: NUMBER NUMBER NUMBER NUMBER
{
    auto model = tech_config->GetLastModel();
    double distance = $1;
    double coupling_cap = $2;
    double fringe_cap = $3;
    double res = $4;
    if (model != nullptr) {
        if (model->tmp_res_over_ != nullptr) {
            model->tmp_res_over_->AddEntry(distance, coupling_cap, fringe_cap, res);
        }
        if (model->tmp_cap_over_ != nullptr) {
            model->tmp_cap_over_->AddEntry(distance, coupling_cap, fringe_cap, res);
        }
        if (model->tmp_cap_under_ != nullptr) {
            model->tmp_cap_under_->AddEntry(distance, coupling_cap, fringe_cap, res);
        }
        if (model->tmp_cap_diagunder_ != nullptr) {
            model->tmp_cap_diagunder_->AddEntry(distance, coupling_cap, fringe_cap, res);
        }
        if (model->tmp_cap_overunder_ != nullptr) {
            model->tmp_cap_overunder_->AddEntry(distance, coupling_cap, fringe_cap, res);
        }
    }
}
;

table_end: TABLE_END
;

end_densitymodel: DENSITYMODEL_END NUMBER
;

%%

void phydb::Parser::error(const location &loc , const std::string &message) {
    cout << "Error: " << message << endl << "Error location: " << driver.location() << endl;
}

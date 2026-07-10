//
// ********************************************************************
// * License and Disclaimer                                           *
// * *
// * The  Geant4 software  is  copyright of the Copyright Holders  of *
// * the Geant4 Collaboration.  It is provided  under  the terms  and *
// * conditions of the Geant4 Software License,  included in the file *
// * LICENSE and available at  http://cern.ch/geant4/license .  These *
// * include a list of copyright holders.                             *
// * *
// * Neither the authors of this software system, nor their employing *
// * institutes,nor the agencies providing financial support for this *
// * work  make  any representation or  warranty, express or implied, *
// * regarding  this  software system or assume any liability for its *
// * use.  Please see the license in the file  LICENSE  and URL above *
// * for the full disclaimer and the limitation of liability.        *
// * *
// * This  code  implementation is the result of  the  scientific and *
// * technical work of the GEANT4 collaboration.                      *
// * By using,  copying,  modifying or  distributing the software (or *
// * any work based  on the software)  you  agree  to acknowledge its *
// * use  in  resulting  scientific  publications,  and indicate your *
// * acceptance of all terms of the Geant4 Software license.          *
// ********************************************************************
//
/// \file DetectorConstruction.cc
/// \brief Implementation of the DetectorConstruction class

#include "DetectorConstruction.hh"
#include "DetectorMessenger.hh"

#include "G4AutoDelete.hh"
#include "G4Box.hh"
#include "G4GeometryManager.hh"
#include "G4GlobalMagFieldMessenger.hh"
#include "G4LogicalVolume.hh"
#include "G4LogicalVolumeStore.hh"
#include "G4Material.hh"
#include "G4NistManager.hh"
#include "G4PVPlacement.hh"
#include "G4PhysicalConstants.hh" // Hatalı .keys.hh uzantısı kaldırıldı
#include "G4PhysicalVolumeStore.hh"
#include "G4RunManager.hh"
#include "G4SolidStore.hh"
#include "G4SystemOfUnits.hh"
#include "G4UniformMagField.hh"
#include "G4UnitsTable.hh"

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

DetectorConstruction::DetectorConstruction()
{
  // Varsayılan Geometri Parametreleri (Cam Zırh Katmanları İçin)
  fAbsorberThickness  = 2.0 * cm;  // 1. Katman (Gama Koruması: Kurşunlu Cam)
  fAbsorber2Thickness = 5.0 * cm;  // 2. Katman (Nötron Koruması: Borosilikat Cam)
  fAbsorberSizeYZ     = 10. * cm;  // Test alanı boyutları genişletildi
  fXposAbs            = 0. * cm;

  // Pointer ilk değer atamaları
  fSolidAbsorber2 = nullptr;
  fLogicAbsorber2 = nullptr;
  fPhysiAbsorber2 = nullptr;

  // Malzemeleri yükle ve zırh katmanlarına ata
  DefineMaterials();
  SetWorldMaterial("G4_Galactic");
  SetAbsorberMaterial("LeadGlass");        // Katman 1: Kurşunlu Cam
  SetAbsorber2Material("BorosilicateGlass"); // Katman 2: Borosilikat Cam

  ComputeGeomParameters();

  // Messenger komutları için arayüzü oluştur
  fDetectorMessenger = new DetectorMessenger(this);
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

DetectorConstruction::~DetectorConstruction()
{
  delete fDetectorMessenger;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::DefineMaterials()
{
  G4String symbol;  
  G4double a, z, density;  

  G4int ncomponents, natoms;
  G4double fractionmass;
  G4double temperature, pressure;

  // Elements
  G4Element* H  = new G4Element("Hydrogen", symbol = "H",  z = 1,  a = 1.01   * g / mole);
  G4Element* Li = new G4Element("Lithium",  symbol = "Li", z = 3,  a = 6.941  * g / mole);
  G4Element* Be = new G4Element("Berilium", symbol = "Be", z = 4,  a = 9.01218* g / mole);
  G4Element* B  = new G4Element("Boron",    symbol = "B",  z = 5,  a = 10.811 * g / mole);
  G4Element* C  = new G4Element("Carbon",   symbol = "C",  z = 6,  a = 12.01  * g / mole);
  G4Element* N  = new G4Element("Nitrogen", symbol = "N",  z = 7,  a = 14.01  * g / mole);
  G4Element* O  = new G4Element("Oxygen",   symbol = "O",  z = 8,  a = 16.00  * g / mole);
  G4Element* F  = new G4Element("Fluorine", symbol = "F",  z = 9,  a = 18.9984* g / mole);
  G4Element* Na = new G4Element("Sodium",   symbol = "Na", z = 11, a = 22.99  * g / mole);
  G4Element* Mg = new G4Element("Magnezyum",symbol = "Mg", z = 12, a = 24.305 * g / mole);
  G4Element* Al = new G4Element("Aliminium",symbol = "Al", z = 13, a = 26.9815386 * g / mole);
  G4Element* Si = new G4Element("Silicon",  symbol = "Si", z = 14, a = 28.09  * g / mole);
  G4Element* P  = new G4Element("Phosohorus",symbol = "P", z = 15, a = 30.973762 * g / mole);
  G4Element* S  = new G4Element("Sulfur",   symbol = "S",  z = 16, a = 32.065 * g / mole);
  G4Element* Cl = new G4Element("Chlorine", symbol = "Cl", z = 17, a = 35.453 * g / mole);
  G4Element* Ar = new G4Element("Argon",    symbol = "Ar", z = 18, a = 39.95  * g / mole);
  G4Element* K  = new G4Element("Potasyum", symbol = "K",  z = 19, a = 39.0983* g / mole);
  G4Element* Ca = new G4Element("Kalsiyum", symbol = "Ca", z = 20, a = 40.078 * g / mole);
  G4Element* Sc = new G4Element("Scandium", symbol = "Sc", z = 21, a = 44.955912 * g / mole);
  G4Element* Ti = new G4Element("Titanium", symbol = "Ti", z = 22, a = 47.867 * g / mole);
  G4Element* V  = new G4Element("Vanadium", symbol = "V",  z = 23, a = 50.9415* g / mole);
  G4Element* Cr = new G4Element("Chrome",   symbol = "Cr", z = 24, a = 51.996 * g / mole);
  G4Element* Mn = new G4Element("Manganese",symbol = "Mn", z = 25, a = 54.938045 * g / mole);
  G4Element* Fe = new G4Element("Iron",     symbol = "Fe", z = 26, a = 55.85  * g / mole);
  G4Element* Co = new G4Element("Cobalt",   symbol = "Co", z = 27, a = 58.933 * g / mole);
  G4Element* Ni = new G4Element("Nickel",   symbol = "Ni", z = 28, a = 58.693 * g / mole);
  G4Element* Cu = new G4Element("Copper",   symbol = "Cu", z = 29, a = 63.55  * g / mole);
  G4Element* Zn = new G4Element("Zinc",     symbol = "Zn", z = 30, a = 65.409 * g / mole);
  G4Element* Rb = new G4Element("Rubidium", symbol = "Rb", z = 37, a = 85.4678 * g / mole);
  G4Element* Zr = new G4Element("Zirconium",symbol = "Zr", z = 40, a = 91.224 * g / mole);
  G4Element* Nb = new G4Element("Niobium",  symbol = "Nb", z = 41, a = 92.90638 * g / mole);
  G4Element* Mo = new G4Element("Molybdenum",symbol = "Mo", z = 42, a = 95.95  * g / mole);
  G4Element* Cd = new G4Element("Cadmium",  symbol = "Cd", z = 48, a = 112.411* g / mole);
  G4Element* Te = new G4Element("Tellurium",symbol = "Te", z = 52, a = 127.60 * g / mole);
  G4Element* Ba = new G4Element("Barium",   symbol = "Ba", z = 56, a = 137.327* g / mole);
  G4Element* Gd = new G4Element("Gadolinium",symbol = "Gd", z = 64, a = 157.259* g / mole);
  G4Element* Er = new G4Element("Erbium",   symbol = "Er", z = 68, a = 167.259* g / mole);
  G4Element* Ta = new G4Element("Tantalum", symbol = "Ta", z = 73, a = 180.94788 * g / mole);
  G4Element* W  = new G4Element("Tungsten", symbol = "W",  z = 74, a = 183.85 * g / mole);
  G4Element* Hf = new G4Element("Hafnium",  symbol = "Hf", z = 72, a = 178.49 * g / mole);
  G4Element* I  = new G4Element("Iodine",   symbol = "I",  z = 53, a = 126.90 * g / mole);
  G4Element* Xe = new G4Element("Xenon",    symbol = "Xe", z = 54, a = 131.29 * g / mole);
  G4Element* Pb = new G4Element("Lead",     symbol = "Pb", z = 82, a = 207.19 * g / mole);
  G4Element* Bi = new G4Element("Bismuth",  symbol = "Bi", z = 83, a = 208.9804 * g / mole);

  // ====================================================================
  // YENİ EKLENEN ZIRH MALZEMELERİ
  // ====================================================================

  // 1. Yüksek Yoğunluklu Kurşunlu Cam (Ağırlıkça %65 PbO, %35 SiO2 - Yoğunluk: 5.2 g/cm3)
  G4Material* LeadGlass = new G4Material("LeadGlass", density = 5.2 * g / cm3, ncomponents = 3);
  LeadGlass->AddElement(Pb, fractionmass = 0.603);
  LeadGlass->AddElement(Si, fractionmass = 0.164);
  LeadGlass->AddElement(O,  fractionmass = 0.233);

  // 2. Borosilikat Cam (Tipik Pyrex bileşimi: %81 SiO2, %13 B2O3, %4 Na2O, %2 Al2O3 - Yoğunluk: 2.23 g/cm3)
  G4Material* BorosilicateGlass = new G4Material("BorosilicateGlass", density = 2.23 * g / cm3, ncomponents = 5);
  BorosilicateGlass->AddElement(Si, fractionmass = 0.378);
  BorosilicateGlass->AddElement(B,  fractionmass = 0.040);
  BorosilicateGlass->AddElement(Na, fractionmass = 0.030);
  BorosilicateGlass->AddElement(Al, fractionmass = 0.011);
  BorosilicateGlass->AddElement(O,  fractionmass = 0.541);

  // ====================================================================

  // Simple materials
  new G4Material("H2Liq", z = 1, a = 1.01 * g / mole, density = 70.8 * mg / cm3);
  new G4Material("Beryllium", z = 4, a = 9.01 * g / mole, density = 1.848 * g / cm3);
  new G4Material("Aluminium", z = 13, a = 26.98 * g / mole, density = 2.700 * g / cm3);
  new G4Material("Silicon", z = 14, a = 28.09 * g / mole, density = 2.330 * g / cm3);

  G4Material* lAr = new G4Material("liquidArgon", density = 1.390 * g / cm3, ncomponents = 1);
  lAr->AddElement(Ar, natoms = 1);

  new G4Material("Iron", z = 26, a = 55.85 * g / mole, density = 7.870 * g / cm3);
  new G4Material("Copper", z = 29, a = 63.55 * g / mole, density = 8.960 * g / cm3);
  new G4Material("Germanium", z = 32, a = 72.61 * g / mole, density = 5.323 * g / cm3);
  new G4Material("Silver", z = 47, a = 107.87 * g / mole, density = 10.50 * g / cm3);
  new G4Material("Tungsten", z = 74, a = 183.85 * g / mole, density = 19.30 * g / cm3);
  new G4Material("Gold", z = 79, a = 196.97 * g / mole, density = 19.32 * g / cm3);
  new G4Material("Lead", z = 82, a = 207.19 * g / mole, density = 11.35 * g / cm3);

  // Tungsten Heavy Alloys
  G4Material* WNIFE95 = new G4Material("WNIFE95", density = 18.0 * g / cm3, ncomponents = 3);
  WNIFE95->AddElement(W, fractionmass = 0.95);
  WNIFE95->AddElement(Ni, fractionmass = 0.04);
  WNIFE95->AddElement(Fe, fractionmass = 0.01);

  G4Material* WNIFE97 = new G4Material("WNIFE97", density = 18.4 * g / cm3, ncomponents = 3);
  WNIFE97->AddElement(W, fractionmass = 0.97);
  WNIFE97->AddElement(Ni, fractionmass = 0.02);
  WNIFE97->AddElement(Fe, fractionmass = 0.01);

  G4Material* WNICU95 = new G4Material("WNICU95", density = 17.9 * g / cm3, ncomponents = 3);
  WNICU95->AddElement(W, fractionmass = 0.95);
  WNICU95->AddElement(Ni, fractionmass = 0.04);
  WNICU95->AddElement(Cu, fractionmass = 0.01);

  G4Material* WNICU97 = new G4Material("WNICU97", density = 18.3 * g / cm3, ncomponents = 3);
  WNICU97->AddElement(W, fractionmass = 0.97);
  WNICU97->AddElement(Ni, fractionmass = 0.02);
  WNICU97->AddElement(Cu, fractionmass = 0.01);

  // Polymer Composites
  G4Material* PLAGdO5 = new G4Material("PLAGdO5", density = 1.074 * g / cm3, ncomponents = 4);
  PLAGdO5->AddElement(C, fractionmass = 0.50408);
  PLAGdO5->AddElement(H, fractionmass = 0.056401);
  PLAGdO5->AddElement(O, fractionmass = 0.4078);
  PLAGdO5->AddElement(Gd, fractionmass = 0.031718);

  G4Material* PLAGdO10 = new G4Material("PLAGdO10", density = 1.117 * g / cm3, ncomponents = 4);
  PLAGdO10->AddElement(C, fractionmass = 0.481167);
  PLAGdO10->AddElement(H, fractionmass = 0.053838);
  PLAGdO10->AddElement(O, fractionmass = 0.4044);
  PLAGdO10->AddElement(Gd, fractionmass = 0.060553);

  G4Material* PLAGdO20 = new G4Material("PLAGdO20", density = 1.2025 * g / cm3, ncomponents = 4);
  PLAGdO20->AddElement(C, fractionmass = 0.44107);
  PLAGdO20->AddElement(H, fractionmass = 0.049350);
  PLAGdO20->AddElement(O, fractionmass = 0.3986);
  PLAGdO20->AddElement(Gd, fractionmass = 0.111014);

  G4Material* ABS0 = new G4Material("ABS0", density = 1.04 * g / cm3, ncomponents = 3);
  ABS0->AddElement(C, fractionmass = 0.817784);
  ABS0->AddElement(H, fractionmass = 0.07625);
  ABS0->AddElement(N, fractionmass = 0.105965);

  G4Material* ABSM5 = new G4Material("ABSM5", density = 1.2313 * g / cm3, ncomponents = 5);
  ABSM5->AddElement(C, fractionmass = 0.778843);
  ABSM5->AddElement(H, fractionmass = 0.072619);
  ABSM5->AddElement(N, fractionmass = 0.100919);
  ABSM5->AddElement(Mo, fractionmass = 0.028541);
  ABSM5->AddElement(S, fractionmass = 0.019077);

  G4Material* ABSM10 = new G4Material("ABSM10", density = 1.4055 * g / cm3, ncomponents = 5);
  ABSM10->AddElement(C, fractionmass = 0.74344);
  ABSM10->AddElement(H, fractionmass = 0.0699319);
  ABSM10->AddElement(N, fractionmass = 0.096332);
  ABSM10->AddElement(Mo, fractionmass = 0.054488);
  ABSM10->AddElement(S, fractionmass = 0.03642);

  G4Material* ABSM20 = new G4Material("ABSM20", density = 1.71 * g / cm3, ncomponents = 5);
  ABSM20->AddElement(C, fractionmass = 0.681487);
  ABSM20->AddElement(H, fractionmass = 0.063542);
  ABSM20->AddElement(N, fractionmass = 0.088304);
  ABSM20->AddElement(Mo, fractionmass = 0.099895);
  ABSM20->AddElement(S, fractionmass = 0.066771);

  G4Material* ABSM30 = new G4Material("ABSM30", density = 1.26 * g / cm3, ncomponents = 5);
  ABSM30->AddElement(C, fractionmass = 0.6290);
  ABSM30->AddElement(H, fractionmass = 0.0586);
  ABSM30->AddElement(N, fractionmass = 0.0815);
  ABSM30->AddElement(Mo, fractionmass = 0.1383);
  ABSM30->AddElement(S, fractionmass = 0.0924);

  G4Material* ABSCA5 = new G4Material("ABSCA5", density = 1.2792 * g / cm3, ncomponents = 6);
  ABSCA5->AddElement(C, fractionmass = 0.778843);
  ABSCA5->AddElement(H, fractionmass = 0.072619);
  ABSCA5->AddElement(N, fractionmass = 0.100919);
  ABSCA5->AddElement(W, fractionmass = 0.030408);
  ABSCA5->AddElement(Ca, fractionmass = 0.006629);
  ABSCA5->AddElement(O, fractionmass = 0.010581);

  G4Material* ABSCA10 = new G4Material("ABSCA10", density = 1.4965 * g / cm3, ncomponents = 6);
  ABSCA10->AddElement(C, fractionmass = 0.74344);
  ABSCA10->AddElement(H, fractionmass = 0.069319);
  ABSCA10->AddElement(N, fractionmass = 0.096332);
  ABSCA10->AddElement(W, fractionmass = 0.058052);
  ABSCA10->AddElement(Ca, fractionmass = 0.012655);
  ABSCA10->AddElement(O, fractionmass = 0.020201);

  G4Material* ABSCA20 = new G4Material("ABSCA20", density = 1.9810 * g / cm3, ncomponents = 6);
  ABSCA20->AddElement(C, fractionmass = 0.681487);
  ABSCA20->AddElement(H, fractionmass = 0.063542);
  ABSCA20->AddElement(N, fractionmass = 0.088304);
  ABSCA20->AddElement(W, fractionmass = 0.106429);
  ABSCA20->AddElement(Ca, fractionmass = 0.023202);
  ABSCA20->AddElement(O, fractionmass = 0.037035);

  G4Material* GD2O3 = new G4Material("GD2O3", density = 3.522 * g / cm3, ncomponents = 4);
  GD2O3->AddElement(C, fractionmass = 0.608137);
  GD2O3->AddElement(H, fractionmass = 0.106149);
  GD2O3->AddElement(Gd, fractionmass = 0.247883);
  GD2O3->AddElement(O, fractionmass = 0.037831);

  G4Material* TPUBI = new G4Material("TPUBI", density = 1.20 * g / cm3, ncomponents = 5);
  TPUBI->AddElement(C, fractionmass = 0.483);
  TPUBI->AddElement(H, fractionmass = 0.0625);
  TPUBI->AddElement(Bi, fractionmass = 0.1667);
  TPUBI->AddElement(O, fractionmass = 0.233);
  TPUBI->AddElement(N, fractionmass = 0.0548);

  G4Material* PVA = new G4Material("PVA", density = 1.25 * g / cm3, ncomponents = 3);
  PVA->AddElement(C, fractionmass = 0.545291);
  PVA->AddElement(H, fractionmass = 0.091518);
  PVA->AddElement(O, fractionmass = 0.363190);

  // Parafin Kütüphanesi
  G4Material* Paraffin = new G4Material("Paraffin", density = 0.93 * g / cm3, ncomponents = 2);
  Paraffin->AddElement(C, fractionmass = 0.8514);
  Paraffin->AddElement(H, fractionmass = 0.1486);

  G4Material* PVAV5 = new G4Material("PVAV5", density = 1.30219 * g / cm3, ncomponents = 5);
  PVAV5->AddElement(C, fractionmass = 0.518026);
  PVAV5->AddElement(H, fractionmass = 0.086942);
  PVAV5->AddElement(O, fractionmass = 0.345030);
  PVAV5->AddElement(Fe, fractionmass = 0.026147);
  PVAV5->AddElement(V, fractionmass = 0.023852);

  G4Material* PVAV10 = new G4Material("PVAV10", density = 1.35893 * g / cm3, ncomponents = 5);
  PVAV10->AddElement(C, fractionmass = 0.490762);
  PVAV10->AddElement(H, fractionmass = 0.082366);
  PVAV10->AddElement(O, fractionmass = 0.326871);
  PVAV10->AddElement(Fe, fractionmass = 0.052295);
  PVAV10->AddElement(V, fractionmass = 0.047704);

  G4Material* PVAV15 = new G4Material("PVAV15", density = 1.42083 * g / cm3, ncomponents = 5);
  PVAV15->AddElement(C, fractionmass = 0.463497);
  PVAV15->AddElement(H, fractionmass = 0.077790);
  PVAV15->AddElement(O, fractionmass = 0.308711);
  PVAV15->AddElement(Fe, fractionmass = 0.078443);
  PVAV15->AddElement(V, fractionmass = 0.071556);

  G4Material* PVAV20 = new G4Material("PVAV20", density = 1.48865 * g / cm3, ncomponents = 5);
  PVAV20->AddElement(C, fractionmass = 0.436233);
  PVAV20->AddElement(H, fractionmass = 0.073214);
  PVAV20->AddElement(O, fractionmass = 0.290552);
  PVAV20->AddElement(Fe, fractionmass = 0.104591);
  PVAV20->AddElement(V, fractionmass = 0.095408);

  G4Material* PVAV25 = new G4Material("PVAV25", density = 1.56327 * g / cm3, ncomponents = 5);
  PVAV25->AddElement(C, fractionmass = 0.408968);
  PVAV25->AddElement(H, fractionmass = 0.068638);
  PVAV25->AddElement(O, fractionmass = 0.272392);
  PVAV25->AddElement(Fe, fractionmass = 0.130739);
  PVAV25->AddElement(V, fractionmass = 0.119260);

  G4Material* PVAV30 = new G4Material("PVAV30", density = 1.64576 * g / cm3, ncomponents = 5);
  PVAV30->AddElement(C, fractionmass = 0.381704);
  PVAV30->AddElement(H, fractionmass = 0.064062);
  PVAV30->AddElement(O, fractionmass = 0.254233);
  PVAV30->AddElement(Fe, fractionmass = 0.156887);
  PVAV30->AddElement(V, fractionmass = 0.143112);

  G4Material* PVANb5 = new G4Material("PVANb5", density = 1.30518 * g / cm3, ncomponents = 5);
  PVANb5->AddElement(C, fractionmass = 0.518026);
  PVANb5->AddElement(H, fractionmass = 0.086942);
  PVANb5->AddElement(O, fractionmass = 0.345030);
  PVANb5->AddElement(Fe, fractionmass = 0.018771);
  PVANb5->AddElement(Nb, fractionmass = 0.031228);

  G4Material* PVANb10 = new G4Material("PVANb10", density = 1.36547 * g / cm3, ncomponents = 5);
  PVANb10->AddElement(C, fractionmass = 0.490762);
  PVANb10->AddElement(H, fractionmass = 0.082366);
  PVANb10->AddElement(O, fractionmass = 0.326871);
  PVANb10->AddElement(Fe, fractionmass = 0.037542);
  PVANb10->AddElement(Nb, fractionmass = 0.062457);

  G4Material* PVANb15 = new G4Material("PVANb15", density = 1.43160 * g / cm3, ncomponents = 5);
  PVANb15->AddElement(C, fractionmass = 0.463497);
  PVANb15->AddElement(H, fractionmass = 0.077790);
  PVANb15->AddElement(O, fractionmass = 0.308711);
  PVANb15->AddElement(Fe, fractionmass = 0.056313);
  PVANb15->AddElement(Nb, fractionmass = 0.093686);

  G4Material* PVANb20 = new G4Material("PVANb20", density = 1.50445 * g / cm3, ncomponents = 5);
  PVANb20->AddElement(C, fractionmass = 0.436233);
  PVANb20->AddElement(H, fractionmass = 0.073214);
  PVANb20->AddElement(O, fractionmass = 0.290552);
  PVANb20->AddElement(Fe, fractionmass = 0.075085);
  PVANb20->AddElement(Nb, fractionmass = 0.124914);

  G4Material* PVANb25 = new G4Material("PVANb25", density = 1.58512 * g / cm3, ncomponents = 5);
  PVANb25->AddElement(C, fractionmass = 0.408968);
  PVANb25->AddElement(H, fractionmass = 0.068638);
  PVANb25->AddElement(O, fractionmass = 0.272392);
  PVANb25->AddElement(Fe, fractionmass = 0.093856);
  PVANb25->AddElement(Nb, fractionmass = 0.156143);

  G4Material* PVANb30 = new G4Material("PVANb30", density = 1.67493 * g / cm3, ncomponents = 5);
  PVANb30->AddElement(C, fractionmass = 0.381704);
  PVANb30->AddElement(H, fractionmass = 0.064062);
  PVANb30->AddElement(O, fractionmass = 0.254233);
  PVANb30->AddElement(Fe, fractionmass = 0.112627);
  PVANb30->AddElement(Nb, fractionmass = 0.187372);

  // Gas definitions & chemical systems
  G4Material* H2O = new G4Material("Water", density = 1.000 * g / cm3, ncomponents = 2);
  H2O->AddElement(H, natoms = 2);
  H2O->AddElement(O, natoms = 1);
  H2O->GetIonisation()->SetMeanExcitationEnergy(78 * eV);

  G4Material* Air = new G4Material("Air", density = 1.290 * mg / cm3, ncomponents = 2);
  Air->AddElement(N, fractionmass = 0.7);
  Air->AddElement(O, fractionmass = 0.3);

  G4Material* Graphite = new G4Material("Graphite", density = 1.7 * g / cm3, ncomponents = 1);
  Graphite->AddElement(C, fractionmass = 1.);

  G4Material* Havar = new G4Material("Havar", density = 8.3 * g / cm3, ncomponents = 5);
  Havar->AddElement(Cr, fractionmass = 0.1785);
  Havar->AddElement(Fe, fractionmass = 0.1822);
  Havar->AddElement(Co, fractionmass = 0.4452);
  Havar->AddElement(Ni, fractionmass = 0.1310);
  Havar->AddElement(W,  fractionmass = 0.0631);

  density     = universe_mean_density;  
  pressure    = 3.e-18 * pascal;
  temperature = 2.73 * kelvin;
  new G4Material("Galactic", z = 1, a = 1.01 * g / mole, density, kStateGas, temperature, pressure);
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::ComputeGeomParameters()
{
  // İki tabaka arka arkaya sıfıra sıfır dizilecek şekilde X konumları hesaplanır.
  // X = 0 noktası tam iki zırh bloğunun birleştiği (arayüz) düzlemdir.
  
  fXstartAbs = -fAbsorberThickness;                 // 1. Zırhın başlangıcı
  fXendAbs   = fAbsorber2Thickness;                 // 2. Zırhın bitişi

  G4double totalLength = fAbsorberThickness + fAbsorber2Thickness;
  fWorldSizeX  = 2.4 * totalLength;
  fWorldSizeYZ = 1.2 * fAbsorberSizeYZ;

  if (nullptr != fPhysiWorld) {
    ChangeGeometry();
  }
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

G4VPhysicalVolume* DetectorConstruction::Construct()
{
  if (nullptr != fPhysiWorld) {
    return fPhysiWorld;
  }

  // World Hacmi
  fSolidWorld = new G4Box("World", fWorldSizeX / 2, fWorldSizeYZ / 2, fWorldSizeYZ / 2);
  fLogicWorld = new G4LogicalVolume(fSolidWorld, fWorldMaterial, "World");
  fPhysiWorld = new G4PVPlacement(0, G4ThreeVector(0., 0., 0.), fLogicWorld, "World", 0, false, 0);

  // 1. TABAKA ABSORBER (Ön Zırh - Kurşunlu Cam)
  fSolidAbsorber = new G4Box("Absorber1", fAbsorberThickness / 2, fAbsorberSizeYZ / 2, fAbsorberSizeYZ / 2);
  fLogicAbsorber = new G4LogicalVolume(fSolidAbsorber, fAbsorberMaterial, "Absorber1");
  fPhysiAbsorber = new G4PVPlacement(0, G4ThreeVector(-fAbsorberThickness / 2, 0., 0.), 
                                     fLogicAbsorber, "Absorber1", fLogicWorld, false, 0);

  // 2. TABAKA ABSORBER (Arka Zırh - Borosilikat Cam)
  fSolidAbsorber2 = new G4Box("Absorber2", fAbsorber2Thickness / 2, fAbsorberSizeYZ / 2, fAbsorberSizeYZ / 2);
  fLogicAbsorber2 = new G4LogicalVolume(fSolidAbsorber2, fAbsorber2Material, "Absorber2");
  fPhysiAbsorber2 = new G4PVPlacement(0, G4ThreeVector(fAbsorber2Thickness / 2, 0., 0.), 
                                      fLogicAbsorber2, "Absorber2", fLogicWorld, false, 0);

  PrintGeomParameters();

  return fPhysiWorld;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::PrintGeomParameters()
{
  G4cout << "\n---------------------------------------------------------" << G4endl;
  G4cout << " The WORLD   is made of " << G4BestUnit(fWorldSizeX, "Length") << " of " << fWorldMaterial->GetName() << G4endl;
  G4cout << " ABSORBER 1  is made of " << G4BestUnit(fAbsorberThickness, "Length") << " of " << fAbsorberMaterial->GetName() << G4endl;
  G4cout << " ABSORBER 2  is made of " << G4BestUnit(fAbsorber2Thickness, "Length") << " of " << fAbsorber2Material->GetName() << G4endl;
  G4cout << "---------------------------------------------------------\n" << G4endl;
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetAbsorberMaterial(const G4String& materialChoice)
{
  G4Material* pttoMaterial = G4NistManager::Instance()->FindOrBuildMaterial(materialChoice);
  if (pttoMaterial && fAbsorberMaterial != pttoMaterial) {
    fAbsorberMaterial = pttoMaterial;
    if (fLogicAbsorber) {
      fLogicAbsorber->SetMaterial(fAbsorberMaterial);
    }
    G4RunManager::GetRunManager()->PhysicsHasBeenModified();
  }
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetAbsorber2Material(const G4String& materialChoice)
{
  G4Material* pttoMaterial = G4NistManager::Instance()->FindOrBuildMaterial(materialChoice);
  if (pttoMaterial && fAbsorber2Material != pttoMaterial) {
    fAbsorber2Material = pttoMaterial;
    if (fLogicAbsorber2) {
      fLogicAbsorber2->SetMaterial(fAbsorber2Material);
    }
    G4RunManager::GetRunManager()->PhysicsHasBeenModified();
  }
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetWorldMaterial(const G4String& materialChoice)
{
  G4Material* pttoMaterial = G4NistManager::Instance()->FindOrBuildMaterial(materialChoice);
  if (pttoMaterial && fWorldMaterial != pttoMaterial) {
    fWorldMaterial = pttoMaterial;
    if (fLogicWorld) {
      fLogicWorld->SetMaterial(fWorldMaterial);
    }
    G4RunManager::GetRunManager()->PhysicsHasBeenModified();
  }
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetAbsorberThickness(G4double val)
{
  fAbsorberThickness = val;
  ComputeGeomParameters();
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetAbsorber2Thickness(G4double val)
{
  fAbsorber2Thickness = val;
  ComputeGeomParameters();
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetAbsorberSizeYZ(G4double val)
{
  fAbsorberSizeYZ = val;
  ComputeGeomParameters();
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetWorldSizeX(G4double val)
{
  fWorldSizeX = val;
  ComputeGeomParameters();
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetWorldSizeYZ(G4double val)
{
  fWorldSizeYZ = val;
  ComputeGeomParameters();
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::SetAbsorberXpos(G4double val)
{
  fXposAbs = val;
  ComputeGeomParameters();
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::ConstructSDandField()
{
  if (fFieldMessenger.Get() == 0) {
    G4ThreeVector fieldValue = G4ThreeVector();
    G4GlobalMagFieldMessenger* msg = new G4GlobalMagFieldMessenger(fieldValue);
    G4AutoDelete::Register(msg);
    fFieldMessenger.Put(msg);
  }
}

//....oooOO0OOooo........oooOO0OOooo........oooOO0OOooo........oooOO0OOooo......

void DetectorConstruction::ChangeGeometry()
{
  if (fSolidWorld) {
    fSolidWorld->SetXHalfLength(fWorldSizeX * 0.5);
    fSolidWorld->SetYHalfLength(fWorldSizeYZ * 0.5);
    fSolidWorld->SetZHalfLength(fWorldSizeYZ * 0.5);
  }

  if (fSolidAbsorber) {
    fSolidAbsorber->SetXHalfLength(fAbsorberThickness * 0.5);
    fSolidAbsorber->SetYHalfLength(fAbsorberSizeYZ * 0.5);
    fSolidAbsorber->SetZHalfLength(fAbsorberSizeYZ * 0.5);
  }

  if (fSolidAbsorber2) {
    fSolidAbsorber2->SetXHalfLength(fAbsorber2Thickness * 0.5);
    fSolidAbsorber2->SetYHalfLength(fAbsorberSizeYZ * 0.5);
    fSolidAbsorber2->SetZHalfLength(fAbsorberSizeYZ * 0.5);
  }
}

/*
** Copyright 2008, Google Inc.
**
** Licensed under the Apache License, Version 2.0 (the "License"); 
** you may not use this file except in compliance with the License. 
** You may obtain a copy of the License at 
**
**     http://www.apache.org/licenses/LICENSE-2.0 
**
** Unless required by applicable law or agreed to in writing, software 
** distributed under the License is distributed on an "AS IS" BASIS, 
** WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. 
** See the License for the specific language governing permissions and 
** limitations under the License.
*/

#ifndef ANDROID_BLUETOOTH_HEADSET_TABLE_H
#define ANDROID_BLUETOOTH_HEADSET_TABLE_H

namespace android {

struct headset_id_name {
    const char *probe;
    const char *name;
    int id;
};

#define INIT_KNOWN_HEADSET(sys, desc) { sys, desc, -1 }

struct headset_id_name KNOWN_HEADSETS[] = {
	INIT_KNOWN_HEADSET("BH_S100", "HTC BH S100"),
	INIT_KNOWN_HEADSET("BH_M100", "HTC BH M100"),
	INIT_KNOWN_HEADSET("Motorola_H500", "Motorola H500"),
	INIT_KNOWN_HEADSET("Nokia_HS_36W", "Nokia HS-36W"),
	INIT_KNOWN_HEADSET("PLT_510vD", "PLT 510v.D"),
	INIT_KNOWN_HEADSET("M2500_Plantronics", "M2500 by Plantronics"),
	INIT_KNOWN_HEADSET("Nokia_HDW_3", "Nokia HDW-3"),
	INIT_KNOWN_HEADSET("HBH_608", "HBH-608"),
	INIT_KNOWN_HEADSET("HBH_DS970", "HBH-DS970"),
	INIT_KNOWN_HEADSET("iTech_BlueBAND", "i.Tech BlueBAND"),
	INIT_KNOWN_HEADSET("Nokia_BH_800", "Nokia BH-800"),
	INIT_KNOWN_HEADSET("Motorola_H700", "Motorola H700"),
	INIT_KNOWN_HEADSET("HTC_BH_M200", "HTC BH M200"),
	INIT_KNOWN_HEADSET("Jabra_JX10", "Jabra JX10"),
	INIT_KNOWN_HEADSET("Plantronics_320", "320Plantronics"),
	INIT_KNOWN_HEADSET("Plantronics_640", "640Plantronics"),
	INIT_KNOWN_HEADSET("Jabra_BT500", "Jabra BT500"),
	INIT_KNOWN_HEADSET("Motorola_HT820", "Motorola HT820"),
	INIT_KNOWN_HEADSET("HBH_IV840", "HBH-IV840"),
	INIT_KNOWN_HEADSET("Plantronics_6XX", "6XXPlantronics"),
	INIT_KNOWN_HEADSET("Plantronics_3XX", "3XXPlantronics"),
	INIT_KNOWN_HEADSET("HBH_PV710", "HBH-PV710"),
	INIT_KNOWN_HEADSET("Motorola_H670", "Motorola H670"),
	INIT_KNOWN_HEADSET("HBM_300", "HBM-300"),
	INIT_KNOWN_HEADSET("Nokia_BH_208", "Nokia BH-208"),
	INIT_KNOWN_HEADSET("Samsung_WEP410", "Samsung WEP410"),
	INIT_KNOWN_HEADSET("Jabra_BT8010", "Jabra BT8010"),
	INIT_KNOWN_HEADSET("Motorola_S9", "Motorola S9"),
	INIT_KNOWN_HEADSET("Jabra_BT620s", "Jabra BT620s"),
	INIT_KNOWN_HEADSET("Nokia_BH_902", "Nokia BH-902"),
	INIT_KNOWN_HEADSET("HBH_DS220", "HBH-DS220"),
	INIT_KNOWN_HEADSET("HBH_DS980", "HBH-DS980"),
};

#undef INIT_KNOWN_HEADSET

}; // namespace android

#endif // ANDROID_BLUETOOTH_HEADSET_TABLE_H

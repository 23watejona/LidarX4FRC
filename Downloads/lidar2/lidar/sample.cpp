#include <ntcore.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>
#include <iostream>
#include <unistd.h>

#include "ntpublish.h"

class Foo {
     public: 
Foo() {};
void bar() {
   m_inst = nt::NetworkTableInstance::GetDefault();
   m_table = m_inst.GetTable("datatable");
   m_inst.StartClient();
   m_inst.SetServer("10.20.0.2");
   m_entry = m_table->GetEntry("x");
}
nt::NetworkTableInstance m_inst;
std::shared_ptr<NetworkTable> m_table;
nt::NetworkTableEntry m_entry;
};

LidarTable *g_table;

int main(int, char **) {

      Foo f; 
	f.bar();
	//auto table = f.m_inst.GetTable("datatable");
	//nt::NetworkTableEntry xEntry = table->GetEntry("x");

	while(1) {
		f.m_entry.SetDouble(2.0);
		std::cout << "ok\n"; 
//		sleep(1);
	}
}

/** Copyright (C) 2022  Frieder Pankratz <frieder.pankratz@gmail.com> **/


#include "TraactDataflowTestPlugin.h"

namespace traact::test {
CREATE_TRAACT_COMPONENT_FACTORY(DataflowTestComponent)
}


BEGIN_TRAACT_PLUGIN_REGISTRATION
    REGISTER_DEFAULT_COMPONENT(traact::test::DataflowTestComponent)
END_TRAACT_PLUGIN_REGISTRATION




#include "hexapod_renderer_builder.h"

#include "phantomx_renderer_simple.h"
#include "phantomx_state_calculator.h"


std::unique_ptr<IHexapodRenderer> HexapodRendererBuilder::Build(
    const std::shared_ptr<const AbstractHexapodStateCalculator>& calculator,
    const DisplayQuality display_quality
    )
{
    //AbstractHexapodStateCalculatorのtypeを見て、適切なrendererを返す
    if (typeid(calculator) == typeid(const std::shared_ptr<const PhantomXStateCalclator>))
    {
        return std::make_unique<PhantomXRendererSimple>(calculator, display_quality);
	}


    return std::make_unique<PhantomXRendererSimple>(calculator, display_quality);
}

#include "hexapod_renderer_builder.h"

#include "phantomx_renderer_model.h"
#include "phantomx_renderer_simple.h"
#include "phantomx_mk2.h"

std::unique_ptr<IHexapodRenderer> HexapodRendererBuilder::Build(
    const std::shared_ptr<const IHexapodCoordinateConverter>& converter_ptr,
    const std::shared_ptr<const IHexapodJointCalculator>& calculator_ptr,
    const DisplayQuality display_quality
)
{
    //! @todo IHexapod��type�����āA�K�؂�renderer��Ԃ�

    if (display_quality == DisplayQuality::kHigh)
    {
        return std::make_unique<PhantomXRendererModel>(converter_ptr, calculator_ptr);
    }
    else
    {
        return std::make_unique<PhantomXRendererSimple>(converter_ptr, calculator_ptr, display_quality);
    }
}

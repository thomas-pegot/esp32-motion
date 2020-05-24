set(COMPONENT_SRCS
  lucas_kanade_opitcal_flow..c
  deflicker.c
  convolution.c
  )

set(COMPONENT_ADD_INCLUDEDIRS
  include
  )


set(COMPONENT_REQUIRES src)

register_component()
set(COMPONENT_SRCS
  lucas_kanade_opitcal_flow.c
  deflicker.c
  convolution.c
  block_matching.c
  motion.c
  epzs.c
  )

set(COMPONENT_ADD_INCLUDEDIRS
  include
  )


register_component()
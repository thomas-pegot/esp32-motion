set(COMPONENT_SRCS
  motion.c
  deflicker.c
  convolution.c
  )

set(COMPONENT_ADD_INCLUDEDIRS
  include
  )


set(COMPONENT_REQUIRES src)

register_component()
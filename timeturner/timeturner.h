#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

typedef enum Registration_Tag {
  Success,
  Failure,
  NoServer,
} Registration_Tag;

typedef struct Registration {
  Registration_Tag tag;
  union {
    struct {
      uint64_t success;
    };
  };
} Registration;

struct Registration retrieve_next_id(void);

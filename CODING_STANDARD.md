# Coding standard: Barr Group Embedded C Coding Standard (BARR-C:2018)

All code in `app_program/` (and Buddy 1's `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c/.h`)
follows BARR-C:2018 as far as it can without changing behaviour or renaming the
interfaces between buddies. This file says what is enforced, how to keep it that
way, and where the code knowingly deviates.

## How it was applied (2026-11-06)

The pass is mechanical and was proven not to change the program: the drive
firmware (`make WIFI=cyw43 NET=lwip MQTT=1`) built before and after the pass gives
a **byte-identical .uf2**, and every object file has the same code and data
(debug line numbers aside; test builds differ only in the `__LINE__` numbers
that `test_report.h` prints). All four build configurations compile with no
warnings, also under `-Wall -Wextra`, and the Buddy 3 and Buddy 5 host tests pass.

| Rule | What it asks | Status |
|---|---|---|
| 1.2.a | Lines at most 80 characters | Code: yes. A few comment tables and ASCII diagrams are wider and were left intact on purpose. |
| 1.3.a, 1.3.b | Braces on every `if`/`else`/`for`/`while`/`do`/`switch` body, each brace on its own line | Yes (clang-format `InsertBraces`, Allman style) |
| 1.8.a | `static` for private functions and data | Yes: T-Kernel's `LOCAL` macro replaced by `static`, `IMPORT` by `extern`, `EXPORT` removed |
| 3.1 | One space after `if`, `while`, `for`, `switch`, `return`; none after a function name | Yes |
| 3.4 | 4-space indents; `case` labels indented inside `switch` | Yes |
| 3.5 | No tab characters | Yes (tabs expanded at 8 columns, then re-indented) |
| 3.6 | Lines end with LF only | Yes |
| 5.2.a | Fixed-width integer types from `<stdint.h>` instead of `int`, `short`, `long`, `unsigned` | Yes for plain C types. Bit-fields keep `unsigned int`, which C requires. |
| 8.6.a | Constant on the left of `==` and `!=` (`if (E_OK == er)`) | Yes, for simple operands (a variable, member or array element against a literal or an UPPER_CASE constant). |

To keep new code in the same shape, run from the repo root:

```
clang-format --style=file:app_program/.clang-format -i <changed files>
```

`tools/barr_c/` holds the scripts that did the pass (`barr_pass.sh` runs all of
them), so a buddy can apply it to a new file.

## Known deviations

These are deliberate. Each one would touch the interfaces between buddies or the
RTOS API, and so needs the owner's agreement first.

- **5.2 T-Kernel types.** `ID`, `ER`, `INT`, `UINT`, `UB`, `W`, `SYSTIM`, `T_C*` and
  similar stay wherever they meet the micro T-Kernel API, because the kernel
  prototypes use them.
- **7.1 Variable names.** The rule asks for prefixes: `g_` for globals, `p_` for
  pointers, `b_` for booleans. Renaming public variables such as `flg_vc`,
  `mbf_summary` and `app_stats` would break other buddies' code and tests, so only
  new code follows the rule.
- **6.1 Module prefixes.** Public functions keep their names: `motion_*`,
  `buddy3_*`, `obstacle_*`, `vc_*` and the `b1_*` functions in `mqtt_utk.c`.
- **6.3 Function-like macros.** Packing helpers such as `S5_FIT` and `VC_FIELD`
  are still macros, because they are used in constant expressions and static
  asserts.
- **2.1 Commented-out code.** Code that had been commented out was not removed,
  because its owners use it while debugging (in `motion_demo.c`, `motion_control.c`
  and `app_main.c`). Convert it to `#if 0` or delete it when it is no longer
  needed.

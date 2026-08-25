# Upstream provenance

The files in `fanuc_driver/` were supplied in the project's `fanucpy.zip`
archive. They are byte-for-byte identical to the corresponding files in
[`torayeff/fanucpy`](https://github.com/torayeff/fanucpy) at commit
`6f000b8c737f9146566fdf2b745b7ed34979dc7a` (checked 2026-08-25).

Upstream fanucpy is licensed under the Apache License 2.0 and attributes the
controller implementation to Agajan Torayev and contributors. The root
`LICENSE` and `NOTICE` files apply to this redistributed bundle.

`karel/mappdk_call_child.kl.inc` is a separate, experimental project patch. It
is not present in the supplied upstream controller bundle and must be reviewed
and translated before controller deployment.

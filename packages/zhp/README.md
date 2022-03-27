# ZHP

A Http server written in [Zig](https://ziglang.org/).

### Features

- A zero-copy parser and aims to compete with these [parser_benchmarks](https://github.com/rust-bakery/parser_benchmarks/tree/master/http)
while still rejecting nonsense requests. It currently runs around ~1000MB/s.
- Regex url routing thanks to [ctregex](https://github.com/alexnask/ctregex.zig)
- Struct based handlers where the method maps to the function name
- A builtin static file handler, error page handler, and not found page handler
- Middleware support
- Parses forms encoded with `multipart/form-data`
- Streaming responses
- Websockets

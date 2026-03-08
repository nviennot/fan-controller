FROM rust:1.86-bookworm AS builder

RUN apt-get update && apt-get install -y \
    libsoapysdr-dev \
    libclang-dev \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app
COPY Cargo.toml Cargo.lock ./
COPY src/ src/

RUN cargo build --release --bin fan-tx

FROM debian:bookworm-slim

RUN apt-get update && apt-get install -y \
    libsoapysdr0.8 \
    soapysdr0.8-module-bladerf \
    soapysdr0.8-module-hackrf \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /app/target/release/fan-tx /usr/local/bin/fan-tx

EXPOSE 8080

ENTRYPOINT ["fan-tx", "--http-server", "0.0.0.0:8080"]

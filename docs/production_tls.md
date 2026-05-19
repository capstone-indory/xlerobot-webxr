# Production TLS Notes

The dev path uses one self-signed certificate for both `:8443` and `:8444`.
Before operating outside the lab network, put a real TLS terminator in front of
the Mac proxy.

## Recommended Shape

Use Caddy with Let's Encrypt DNS-01:

```caddyfile
webxr.example.com {
    tls {
        dns <provider> <token>
    }

    reverse_proxy /signaling/* 127.0.0.1:8444
    reverse_proxy 127.0.0.1:8443
}
```

Then run the proxy bound to localhost only:

```bash
python3 tools/mac_proxy.py --host 127.0.0.1
```

Keep ZMQ pose PUB restricted to Tailnet or an equivalent private network, for
example `--zmq-addr tcp://100.x.y.z:7001`.

## Dev Cert Sequence

For self-signed development certificates, Quest Browser needs to trust both
origins:

1. Visit `https://<mac-lan-ip>:8444/` and accept the certificate exception.
2. Visit `https://<mac-lan-ip>:8443/?robot=0` and start WebXR.

The first page is only a certificate stub for the WebRTC signaling origin.

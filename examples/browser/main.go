//go:build !js

// The browser example builds the Go physics demo as a self-contained HTML file
// or serves it locally. No JavaScript package manager or network access is needed.
package main

import (
	"context"
	"embed"
	"encoding/base64"
	"flag"
	"fmt"
	"log"
	"net"
	"net/http"
	"os"
	"os/exec"
	"os/signal"
	"path/filepath"
	"strings"
	"syscall"
	"time"
)

//go:generate go run . -export demo.html

//go:embed web/*
var assets embed.FS

func main() {
	addr := flag.String("addr", "127.0.0.1:8080", "local HTTP listen address")
	export := flag.String("export", "", "write a standalone HTML file and exit")
	flag.Parse()
	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()
	if err := run(ctx, *addr, *export); err != nil {
		log.Fatal(err)
	}
}

func run(ctx context.Context, addr, export string) error {
	log.Println("Building the Go WebAssembly demo…")
	html, err := buildDemo(ctx)
	if err != nil {
		return err
	}
	if export != "" {
		if err := os.WriteFile(export, html, 0o644); err != nil {
			return err
		}
		log.Printf("Wrote %s (%d bytes). Open it directly in a browser, including offline.", export, len(html))
		return nil
	}
	listener, err := net.Listen("tcp", addr)
	if err != nil {
		return err
	}
	server := &http.Server{
		Handler: demoHandler(html), ReadHeaderTimeout: 5 * time.Second,
		WriteTimeout: 30 * time.Second, IdleTimeout: time.Minute,
	}
	go func() {
		<-ctx.Done()
		shutdown, cancel := context.WithTimeout(context.Background(), 5*time.Second)
		defer cancel()
		_ = server.Shutdown(shutdown)
	}()
	log.Printf("Open http://%s — press Ctrl+C to stop", listener.Addr())
	if err := server.Serve(listener); err != nil && err != http.ErrServerClosed {
		return err
	}
	return nil
}

func demoHandler(html []byte) http.Handler {
	return http.HandlerFunc(func(w http.ResponseWriter, r *http.Request) {
		if r.URL.Path != "/" && r.URL.Path != "/demo.html" {
			http.NotFound(w, r)
			return
		}
		if r.Method != http.MethodGet && r.Method != http.MethodHead {
			w.Header().Set("Allow", "GET, HEAD")
			http.Error(w, "method not allowed", http.StatusMethodNotAllowed)
			return
		}
		w.Header().Set("Content-Type", "text/html; charset=utf-8")
		w.Header().Set("Cache-Control", "no-store")
		if r.Method == http.MethodGet {
			_, _ = w.Write(html)
		}
	})
}

func moduleRoot(start string) (string, error) {
	dir, err := filepath.Abs(start)
	if err != nil {
		return "", err
	}
	for {
		if data, err := os.ReadFile(filepath.Join(dir, "go.mod")); err == nil && strings.Contains(string(data), "module github.com/kyrqon/GoSimPhysics") {
			return dir, nil
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			return "", fmt.Errorf("run this command from the GoSimPhysics checkout")
		}
		dir = parent
	}
}

func buildDemo(ctx context.Context) ([]byte, error) {
	root, err := moduleRoot(".")
	if err != nil {
		return nil, err
	}
	temp, err := os.MkdirTemp("", "gosim-browser-*")
	if err != nil {
		return nil, err
	}
	defer os.RemoveAll(temp)
	goExe, err := exec.LookPath("go")
	if err != nil {
		return nil, fmt.Errorf("Go 1.27 or later is required to rebuild the demo: %w", err)
	}
	toolchain := exec.CommandContext(ctx, goExe, "env", "GOROOT")
	toolchain.Dir = root
	goRoot, err := toolchain.Output()
	if err != nil {
		return nil, fmt.Errorf("locate Go runtime: %w", err)
	}
	wasmFile := filepath.Join(temp, "physics.wasm")
	compile := exec.CommandContext(ctx, goExe, "build", "-trimpath", "-buildvcs=false", "-ldflags=-s -w", "-o", wasmFile, "./examples/browser/wasm")
	compile.Dir = root
	compile.Env = append(os.Environ(), "GOOS=js", "GOARCH=wasm", "CGO_ENABLED=0")
	if output, err := compile.CombinedOutput(); err != nil {
		return nil, fmt.Errorf("compile WebAssembly: %w\n%s", err, output)
	}
	wasm, err := os.ReadFile(wasmFile)
	if err != nil {
		return nil, err
	}
	runtimeJS, err := os.ReadFile(filepath.Join(strings.TrimSpace(string(goRoot)), "lib", "wasm", "wasm_exec.js"))
	if err != nil {
		return nil, fmt.Errorf("read matching Go WebAssembly runtime: %w", err)
	}
	return bundleHTML(wasm, runtimeJS)
}

func bundleHTML(wasm, runtimeJS []byte) ([]byte, error) {
	if len(wasm) < 8 || string(wasm[:4]) != "\x00asm" {
		return nil, fmt.Errorf("invalid WebAssembly binary")
	}
	read := func(name string) (string, error) {
		data, err := assets.ReadFile("web/" + name)
		return string(data), err
	}
	html, err := read("index.html")
	if err != nil {
		return nil, err
	}
	css, err := read("style.css")
	if err != nil {
		return nil, err
	}
	app, err := read("app.js")
	if err != nil {
		return nil, err
	}
	replacements := map[string]string{
		"<!-- INLINE_STYLE -->": "<style>" + css + "</style>",
		"<!-- WASM_RUNTIME -->": "<script>" + string(runtimeJS) + "</script>",
		"<!-- WASM_BINARY -->":  "<script id=\"gosim-wasm\" type=\"application/octet-stream\">" + base64.StdEncoding.EncodeToString(wasm) + "</script>",
		"<!-- INLINE_APP -->":   "<script>" + app + "</script>",
	}
	for marker, content := range replacements {
		if strings.Count(html, marker) != 1 {
			return nil, fmt.Errorf("HTML template must contain exactly one %s", marker)
		}
		html = strings.Replace(html, marker, content, 1)
	}
	return []byte("<!-- Generated with go generate ./examples/browser; do not edit. Go runtime: see THIRD_PARTY_LICENSES.txt. -->\n" + html), nil
}

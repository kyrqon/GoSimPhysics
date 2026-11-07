//go:build !js

package main

import (
	"bytes"
	"encoding/base64"
	"net/http"
	"net/http/httptest"
	"os"
	"path/filepath"
	"strings"
	"testing"
)

func TestStandaloneBundleEmbedsRuntimeAndBinary(t *testing.T) {
	wasm := []byte{0, 'a', 's', 'm', 1, 0, 0, 0}
	html, err := bundleHTML(wasm, []byte("/* matching Go runtime */"))
	if err != nil {
		t.Fatal(err)
	}
	for _, marker := range []string{"<!-- INLINE_STYLE -->", "<!-- INLINE_APP -->", "<!-- WASM_RUNTIME -->", "<!-- WASM_BINARY -->"} {
		if bytes.Contains(html, []byte(marker)) {
			t.Fatalf("unresolved bundle placeholder %s", marker)
		}
	}
	if !bytes.Contains(html, []byte(base64.StdEncoding.EncodeToString(wasm))) || !bytes.Contains(html, []byte("matching Go runtime")) {
		t.Fatal("offline bundle omitted its binary or runtime")
	}
	for _, external := range []string{"<script src=", "<link rel=\"stylesheet\"", "https://cdn.", "fetch("} {
		if bytes.Contains(html, []byte(external)) {
			t.Fatalf("standalone bundle depends on external resources: %s", external)
		}
	}
	if _, err := bundleHTML([]byte("invalid"), nil); err == nil {
		t.Fatal("accepted an invalid wasm build")
	}
}

func TestLocalDemoRoutes(t *testing.T) {
	content := []byte("<!doctype html><title>GoSimPhysics</title>")
	handler := demoHandler(content)
	for _, path := range []string{"/", "/demo.html"} {
		for _, method := range []string{http.MethodGet, http.MethodHead} {
			response := httptest.NewRecorder()
			handler.ServeHTTP(response, httptest.NewRequest(method, path, nil))
			if response.Code != http.StatusOK || !strings.HasPrefix(response.Header().Get("Content-Type"), "text/html") {
				t.Fatalf("%s %s returned %d", method, path, response.Code)
			}
			if method == http.MethodGet && !bytes.Equal(response.Body.Bytes(), content) {
				t.Fatal("server changed the standalone document")
			}
			if method == http.MethodHead && response.Body.Len() != 0 {
				t.Fatal("HEAD returned a body")
			}
		}
	}
	for _, test := range []struct {
		method, path string
		status       int
	}{{http.MethodPost, "/", http.StatusMethodNotAllowed}, {http.MethodGet, "/missing", http.StatusNotFound}} {
		response := httptest.NewRecorder()
		handler.ServeHTTP(response, httptest.NewRequest(test.method, test.path, nil))
		if response.Code != test.status {
			t.Fatalf("%s %s returned %d, want %d", test.method, test.path, response.Code, test.status)
		}
	}
}

func TestModuleRootFromNestedCheckout(t *testing.T) {
	root, err := moduleRoot("web")
	if err != nil {
		t.Fatalf("cannot find repository from nested path: %q, %v", root, err)
	}
	module, err := os.ReadFile(filepath.Join(root, "go.mod"))
	if err != nil || !strings.Contains(string(module), "module github.com/kyrqon/GoSimPhysics") {
		t.Fatal("resolved a different module")
	}
	if _, err := moduleRoot(t.TempDir()); err == nil {
		t.Fatal("accepted a directory outside the checkout")
	}
}

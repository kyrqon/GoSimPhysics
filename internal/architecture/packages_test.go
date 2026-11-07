package architecture_test

import (
	"go/parser"
	"go/token"
	"io/fs"
	"os"
	"path/filepath"
	"strconv"
	"strings"
	"testing"
)

const modulePath = "github.com/kyrqon/GoSimPhysics"

// Dependencies must point from dynamics to collision to math. Go catches cycles,
// while this test also rejects acyclic shortcuts that break those boundaries.
func TestPublicPackageLayers(t *testing.T) {
	root := filepath.Join("..", "..")
	layers := map[string][]string{
		"math2d":      {"internal/numeric"},
		"math3d":      {"internal/numeric"},
		"collision2d": {"math2d", "internal/numeric"},
		"collision3d": {"math3d", "internal/numeric"},
		"dynamics2d":  {"math2d", "collision2d", "internal/numeric"},
		"dynamics3d":  {"math3d", "collision3d", "internal/numeric"},
	}
	for name, allowed := range layers {
		entries, err := os.ReadDir(filepath.Join(root, name))
		if err != nil {
			t.Fatal(err)
		}
		for _, entry := range entries {
			if entry.IsDir() || !strings.HasSuffix(entry.Name(), ".go") || strings.HasSuffix(entry.Name(), "_test.go") {
				continue
			}
			filename := filepath.Join(root, name, entry.Name())
			for _, imported := range imports(t, filename) {
				if !strings.HasPrefix(imported, modulePath+"/") {
					continue
				}
				relative := strings.TrimPrefix(imported, modulePath+"/")
				ok := false
				for _, permitted := range allowed {
					ok = ok || relative == permitted
				}
				if !ok {
					t.Errorf("%s imports %s across its package boundary", filename, imported)
				}
			}
		}
	}
}

func TestExamplesAndTestsUseCanonicalImports(t *testing.T) {
	root := filepath.Join("..", "..")
	err := filepath.WalkDir(root, func(path string, entry fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if entry.IsDir() {
			if entry.Name() == ".git" || entry.Name() == "vendor" || entry.Name() == "node_modules" {
				return filepath.SkipDir
			}
			return nil
		}
		if !strings.HasSuffix(path, ".go") {
			return nil
		}
		for _, imported := range imports(t, path) {
			if imported == modulePath || imported == modulePath+"/physics3d" {
				t.Errorf("%s still imports removed package %s", path, imported)
			}
		}
		return nil
	})
	if err != nil {
		t.Fatal(err)
	}
}

func imports(t *testing.T, filename string) []string {
	t.Helper()
	file, err := parser.ParseFile(token.NewFileSet(), filename, nil, parser.ImportsOnly)
	if err != nil {
		t.Fatal(err)
	}
	var result []string
	for _, spec := range file.Imports {
		value, err := strconv.Unquote(spec.Path.Value)
		if err != nil {
			t.Fatal(err)
		}
		result = append(result, value)
	}
	return result
}

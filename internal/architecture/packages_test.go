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

// Each physics core owns its dimension's math, geometry, and dynamics. Ballistics
// depends on those cores; the cores must not depend on optional projectile rules.
// Go catches cycles, while this test also rejects shortcuts across dimensions.
func TestPublicPackageLayers(t *testing.T) {
	root := filepath.Join("..", "..")
	layers := map[string][]string{
		"physics2d":            {"internal/numeric"},
		"physics3d":            {"internal/numeric"},
		"ballistics":           {"internal/numeric"},
		"ballistics2d":         {"ballistics", "physics2d", "internal/numeric", "internal/penetration"},
		"ballistics3d":         {"ballistics", "physics3d", "internal/numeric", "internal/penetration"},
		"internal/penetration": {"ballistics", "internal/numeric"},
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
			switch imported {
			case modulePath,
				modulePath + "/math2d", modulePath + "/math3d",
				modulePath + "/collision2d", modulePath + "/collision3d",
				modulePath + "/dynamics2d", modulePath + "/dynamics3d":
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

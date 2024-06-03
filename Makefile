export SHELL:=/bin/bash

export REPO_ROOT := $(shell pwd)

all: build test compile-commands

.PHONY: build-image
build-image:
	docker compose -f docker/compose.yaml build

.PHONY: up
up:
	docker compose -f docker/compose.yaml up --detach drake

.PHONY: enter
enter:
	docker compose -f docker/compose.yaml exec --user ${USER} --interactive --tty drake bash

.PHONY: down
down:
	docker compose -f docker/compose.yaml down

.PHONY: clean
clean:
	bazel clean

.PHONY: build
build:
	bazel build --config=clang //...

.PHONY: test
test: build
	bazel test --config=clang //...

.PHONY: compile-commands
compile-commands:
	bazel run @hedron_compile_commands//:refresh_all

.PHONY: tidy
tidy:
	find . \( -name "*.c" -o -name "*.h" -o -name "*.hpp" -o -name "*.cpp" -o -name "*.cc" \) | xargs clang-tidy --config-file=.clang-tidy

.PHONY: format
format:
	find . \( -name "*.c" -o -name "*.h" -o -name "*.hpp" -o -name "*.cpp" -o -name "*.cc" \) | xargs clang-format -i -style=file

.PHONY: buildifier
buildifier:
	bazel-bin/tools/lint/buildifier --all # Reformat all Bazel files.

# Temporary
################################################################################
.PHONY: build-glider
build-glider:
	bazel build --config=clang //examples/glider

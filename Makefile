export SHELL:=/bin/bash

export REPO_ROOT := $(shell pwd)
export GID := $(shell stat -c "%g" .)
export UID := $(shell stat -c "%u" .)

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
	bazel run --config=clang @hedron_compile_commands//:refresh_all

.PHONY: tidy
tidy:
	find . \( -name "*.c" -o -name "*.h" -o -name "*.hpp" -o -name "*.cpp" -o -name "*.cc" \) | xargs clang-tidy --config-file=.clang-tidy

.PHONY: format
format:
	find . \( -name "*.c" -o -name "*.h" -o -name "*.hpp" -o -name "*.cpp" -o -name "*.cc" \) | xargs clang-format-15 -i -style=file

.PHONY: buildifier
buildifier:
	bazel-bin/tools/lint/buildifier --all # Reformat all Bazel files.

.PHONY: quadrotor
quadrotor:
	bazel run --config=clang //examples/quadrotor:run_quadrotor_dynamics

.PHONY: meshcat-server
meshcat-server:
	bazel run --config=clang //tools:meldis -- --open-window &

.PHONY: venv
venv:
ifeq (${IN_DOCKER_CONTAINER},1)
	uv venv .cvenv
else
	uv venv
endif

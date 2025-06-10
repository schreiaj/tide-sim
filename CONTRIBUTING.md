# Contributing to Tide Sim

Thank you for your interest in contributing to Tide Sim! This document provides guidelines and instructions for contributing.

## Development Setup

1. Fork and clone the repository
2. Create a virtual environment and install dependencies:
```bash
make install
```

3. Install pre-commit hooks:
```bash
pip install pre-commit
pre-commit install
```

## Development Workflow

1. Create a new branch for your feature/fix
2. Make your changes
3. Run the tests:
```bash
make test
```

4. Ensure code quality:
```bash
make lint
make format
```

5. Submit a pull request

## Common Tasks

The project includes a Makefile with common development tasks:

- `make install`: Create virtual environment and install dependencies
- `make test`: Run tests with coverage
- `make lint`: Run linters (ruff and mypy)
- `make lint-fix`: Run ruff with auto-fix enabled
- `make format`: Format code (black and isort)
- `make clean`: Clean up build artifacts and caches

## Code Style

- We use `black` for code formatting
- `isort` for import sorting
- `ruff` for linting
- `mypy` for type checking

## Testing

- Write tests for new features
- Ensure all tests pass
- Maintain or improve test coverage

## Documentation

- Update docstrings for new functions/classes
- Update README.md if needed
- Add examples for new features

## Pull Request Process

1. Update the README.md with details of changes if needed
2. Update the version number in pyproject.toml
3. The PR will be merged once you have the sign-off of at least one other developer 
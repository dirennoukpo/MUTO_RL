ENV_DIR = docker/config
USER_NAME := $(shell whoami)
BASE_ENV = .env.base
PI_ENV = .env.raspberrypi
JETSON_ENV = .env.jetson_nano
REQUIRED_ENVS = $(BASE_ENV) $(PI_ENV) $(JETSON_ENV)

ifeq ($(USER_NAME),jetson)
PROFILE ?= $(JETSON_ENV)
else
PROFILE ?= $(PI_ENV)
endif

# Récupération dynamique du nom du container depuis le .env choisi
NAME = $(shell grep '^CONTAINER_NAME=' $(ENV_DIR)/$(PROFILE) | cut -d'=' -f2)

# Commande de base incluant le fichier .env spécifique
COMPOSE = DISPLAY=$${DISPLAY:-:0} ENV_PROFILE=$(PROFILE) docker compose -f docker/docker-compose.yml --env-file $(ENV_DIR)/$(BASE_ENV) --env-file $(ENV_DIR)/$(PROFILE)

validate-envs:
	@for f in $(REQUIRED_ENVS); do \
		if [ ! -f "$(ENV_DIR)/$$f" ]; then \
			echo "ERREUR: fichier manquant $(ENV_DIR)/$$f"; \
			exit 1; \
		fi; \
	done
	@if [ "$(PROFILE)" != "$(PI_ENV)" ] && [ "$(PROFILE)" != "$(JETSON_ENV)" ]; then \
		echo "ERREUR: PROFILE doit etre $(PI_ENV) ou $(JETSON_ENV)"; \
		exit 1; \
	fi

# Cibles
status: validate-envs
	@echo "User: $(USER_NAME)"
	@echo "Base: $(ENV_DIR)/$(BASE_ENV)"
	@echo "Profile: $(ENV_DIR)/$(PROFILE)"
	@echo "Required: $(ENV_DIR)/$(BASE_ENV), $(ENV_DIR)/$(PI_ENV), $(ENV_DIR)/$(JETSON_ENV)"
	@echo "Container: $(NAME)"

build: validate-envs
	$(COMPOSE) build

run: validate-envs
	-xhost +local:docker 2>/dev/null || true
	$(COMPOSE) up -d

shell:
	docker exec -it $(NAME) bash

clean: validate-envs
	$(COMPOSE) down
	rm -rf build/ install/ log/

build-ros:
	docker exec -it $(NAME) colcon build --symlink-install

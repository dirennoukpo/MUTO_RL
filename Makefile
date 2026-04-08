ENV_DIR = docker/config
USER_NAME := $(shell whoami)
HOSTNAME_HINT := $(shell hostname 2>/dev/null | tr '[:upper:]' '[:lower:]')
BASE_ENV = .env.base
PI_ENV = .env.raspberrypi
JETSON_ENV = .env.jetson_nano
DEFAULT_ENV = .env.default
USER_ENV = .env.$(USER_NAME)
ALL_ENV_FILES = $(notdir $(wildcard $(ENV_DIR)/.env.*))
IGNORED_ENV_FILES = $(BASE_ENV) .env.user $(PI_ENV) $(JETSON_ENV) $(DEFAULT_ENV) $(USER_ENV)
CUSTOM_ENV_FILES = $(filter-out $(IGNORED_ENV_FILES),$(ALL_ENV_FILES))
CUSTOM_ENV = $(firstword $(CUSTOM_ENV_FILES))
REQUIRED_ENVS = $(BASE_ENV) $(PI_ENV) $(JETSON_ENV) $(DEFAULT_ENV)

PROFILE ?=
ifeq ($(strip $(PROFILE)),)
ifneq (,$(findstring jetson,$(HOSTNAME_HINT)))
PROFILE := $(JETSON_ENV)
else ifneq (,$(findstring pi,$(HOSTNAME_HINT)))
PROFILE := $(PI_ENV)
else ifneq (,$(wildcard $(ENV_DIR)/$(USER_ENV)))
PROFILE := $(USER_ENV)
else ifneq (,$(strip $(CUSTOM_ENV)))
PROFILE := $(CUSTOM_ENV)
else
PROFILE := $(DEFAULT_ENV)
endif
endif

# Récupération dynamique du nom du container depuis le .env choisi
NAME = $(shell grep '^CONTAINER_NAME=' $(ENV_DIR)/$(PROFILE) | cut -d'=' -f2)

# Commande de base incluant le fichier .env spécifique
COMPOSE = DISPLAY=$${DISPLAY:-:0} ENV_PROFILE=$(PROFILE) docker compose -f docker/docker-compose.yml --env-file $(ENV_DIR)/$(BASE_ENV) --env-file $(ENV_DIR)/$(PROFILE)

define RUN_WITH_DOCKER_ACCESS
	@if docker info >/dev/null 2>&1; then \
		$(1); \
	elif getent group docker | grep -qw "$(USER_NAME)"; then \
		echo "INFO: session sans groupe docker actif, execution via sg docker"; \
		sg docker -c '$(1)'; \
	else \
		echo "ERREUR: utilisateur $(USER_NAME) absent du groupe docker"; \
		echo "Lancez: sudo usermod -aG docker $(USER_NAME), puis reconnectez-vous"; \
		exit 1; \
	fi
endef

validate-envs:
	@for f in $(REQUIRED_ENVS); do \
		if [ ! -f "$(ENV_DIR)/$$f" ]; then \
			echo "ERREUR: fichier manquant $(ENV_DIR)/$$f"; \
			exit 1; \
		fi; \
	done
	@if [ ! -f "$(ENV_DIR)/$(PROFILE)" ]; then \
		echo "ERREUR: profile introuvable $(ENV_DIR)/$(PROFILE)"; \
		echo "Ajoutez ce fichier ou utilisez PROFILE=$(DEFAULT_ENV)"; \
		exit 1; \
	fi

# Cibles
status: validate-envs
	@echo "User: $(USER_NAME)"
	@echo "Hostname: $(HOSTNAME_HINT)"
	@echo "Base: $(ENV_DIR)/$(BASE_ENV)"
	@echo "Profile: $(ENV_DIR)/$(PROFILE)"
	@echo "Required: $(ENV_DIR)/$(BASE_ENV), $(ENV_DIR)/$(PI_ENV), $(ENV_DIR)/$(JETSON_ENV), $(ENV_DIR)/$(DEFAULT_ENV)"
	@echo "Container: $(NAME)"

build: validate-envs
	$(call RUN_WITH_DOCKER_ACCESS,$(COMPOSE) build)

run: validate-envs
	-xhost +local:docker 2>/dev/null || true
	$(call RUN_WITH_DOCKER_ACCESS,$(COMPOSE) up -d)

run-clean: validate-envs
	-xhost +local:docker 2>/dev/null || true
	$(call RUN_WITH_DOCKER_ACCESS,$(COMPOSE) up -d --remove-orphans)

shell:
	$(call RUN_WITH_DOCKER_ACCESS,cid="$$($(COMPOSE) ps -q ros-dev)"; \
	if [ -z "$$cid" ]; then \
		$(COMPOSE) up -d; \
		cid="$$($(COMPOSE) ps -q ros-dev)"; \
	fi; \
	if [ -z "$$cid" ]; then \
		echo "ERREUR: conteneur du service ros-dev introuvable"; \
		exit 1; \
	fi; \
	docker exec -it "$$cid" bash)

clean: validate-envs
	$(call RUN_WITH_DOCKER_ACCESS,$(COMPOSE) down)
	rm -rf build/ install/ log/

build-ros:
	$(call RUN_WITH_DOCKER_ACCESS,cid="$$($(COMPOSE) ps -q ros-dev)"; \
	if [ -z "$$cid" ]; then \
		echo "ERREUR: conteneur du service ros-dev introuvable. Lancez d'abord make run"; \
		exit 1; \
	fi; \
	docker exec -it "$$cid" colcon build --symlink-install)

# Variables
COMPOSE = DISPLAY=$${DISPLAY:-:0} docker compose -f docker/docker-compose.yml
NAME = ros2_humble_tests

build: ## Build l'image sans erreur DISPLAY
	$(COMPOSE) build

run: ## Lance le conteneur avec support X11 (ignore l'erreur si SSH pur)
	-xhost +local:docker 2>/dev/null || true
	$(COMPOSE) up -d

shell: ## Entre dans le container
	docker exec -it $(NAME) bash

clean: ## Nettoyage
	$(COMPOSE) down
	rm -rf build/ install/ log/

build-ros: ## Compiler les drivers à l'intérieur
	docker exec -it $(NAME) colcon build --symlink-install

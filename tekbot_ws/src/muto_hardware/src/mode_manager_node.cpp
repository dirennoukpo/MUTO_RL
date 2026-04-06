/**
 * @file mode_manager_node.cpp
 * @brief Gestionnaire d'États Fini (FSM) centralisé du système MUTO RS.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * ARCHITECTURE
 * ════════════════════════════════════════════════════════════════════════════════
 * Machine cible : Raspberry Pi (ROBOT_ROLE=DRIVER)
 *
 * Ce nœud centralise la logique FSM pour gérer les transitions d'état du robot.
 * IL est lancé par pi_full.launch.py, AVANT le démarrage d'autres nœuds critiques.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * RÔLE EXACT DU FICHIER
 * ════════════════════════════════════════════════════════════════════════════════
 * Implémente FSM (Finite State Machine) avec 7 états :
 *   INIT       → État interne de démarrage (non accessible en entree operateur)
 *   IDLE       → Robot au repos, attendant instruction
 *   DRY_RUN    → Exécution sans armement (test logique, sans couple)
 *   RL_ACTIVE  → Modèle RL en exécution temps réel (armed, moteurs actifs)
 *   SAFE       → Robot désarmé, position de repos, pas de commandes
 *   MANUAL     → Pilotage manuel (admin/développeur via teleoperation)
 *   EMERGENCY  → État absorbing, pas de transition possible à partir de là
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * MATRICE DE TRANSITIONS (explicite pour éviter toute ambiguïté)
 * ════════════════════════════════════════════════════════════════════════════════
 * INIT        → { IDLE }
 * IDLE        → { DRY_RUN, RL_ACTIVE, MANUAL, EMERGENCY }
 * DRY_RUN     → { IDLE, SAFE }
 * RL_ACTIVE   → { SAFE, MANUAL, EMERGENCY }
 * SAFE        → { IDLE }
 * MANUAL      → { IDLE, SAFE }
 * EMERGENCY   → { } (absorbing state)
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * GARDE-FOUS POUR RL_ACTIVE
 * ════════════════════════════════════════════════════════════════════════════════
 * Avant de permettre la transition IDLE → RL_ACTIVE, on vérifie que les 3 validations
 * sont marquées "true" dans le model_card (${WORKING_DIR}/models/model_card_v003.json) :
 *   • dry_run_validated     : test complet du canal DRY_RUN sans erreurs
 *   • sim_real_validated    : validation sim-to-real avec trajectoires identiques
 *   • sim_real_cross_corr_validated : corrélation croisée sim vs réel > seuil
 * Si l'une est false, la demande est refusée avec message d'erreur explicit.
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * SERVICE ROS 2
 * ════════════════════════════════════════════════════════════════════════════════
 * Service: /mode_request [muto_msgs::srv::ModeRequest]
 *   Request  : requested_mode (str, ex: "RL_ACTIVE")
 *   Response : success (bool), message (str), current_mode (str)
 *
 * Topic publish (RT QoS, 200 Hz nominal):
 *   /system_mode [std_msgs::msg::String] → publié à chaque changement d'état
 *
 * ════════════════════════════════════════════════════════════════════════════════
 * MAINTENANCE
 * ════════════════════════════════════════════════════════════════════════════════
 * • Lors d'ajout de nouvel état : ajouter à enum SystemMode, à transitions_ et to_string()
 * • Modéliser les scénarios de transition par test (ex: test_mode_transitions.py)
 * • Déclarer les préconditions explicites (gardes_foo()) avec leur raison
 * • Éviter les modifications de l'ordre de la matrice transitions_
 */


#include "muto_hardware/mode_manager_node.hpp"

#include <cstdlib>
#include <fstream>

namespace muto_hardware {

// Initialise le noeud de gestion des modes et sa matrice de transitions explicite.
ModeManagerNode::ModeManagerNode() : Node("mode_manager_node") {
  // Résolution du chemin depuis la variable d'environnement WORKING_DIR
  // injectée par Docker via docker/config/.env.raspberrypi.
  const char* working_dir = std::getenv("WORKING_DIR");
  if (!working_dir || std::string(working_dir).empty()) {
    RCLCPP_FATAL(
        this->get_logger(),
        "WORKING_DIR non défini. Ce noeud doit tourner dans le container Docker. "
        "Vérifiez docker/config/.env.raspberrypi.");
    throw std::runtime_error("WORKING_DIR non défini");
  }
  std::string default_model_card = std::string(working_dir) + "/models/model_card_v003.json";
  model_card_path_ = declare_parameter<std::string>(
      "model_card_path", default_model_card);

  // Cette matrice formalise les transitions autorisees pour eviter tout saut d'etat implicite.
  transitions_ = {
      {SystemMode::INIT, {SystemMode::IDLE}},
      {SystemMode::IDLE, {SystemMode::DRY_RUN, SystemMode::RL_ACTIVE, SystemMode::MANUAL, SystemMode::EMERGENCY}},
      {SystemMode::DRY_RUN, {SystemMode::IDLE, SystemMode::SAFE}},
      {SystemMode::RL_ACTIVE, {SystemMode::SAFE, SystemMode::MANUAL, SystemMode::EMERGENCY}},
      {SystemMode::SAFE, {SystemMode::IDLE}},
      {SystemMode::MANUAL, {SystemMode::IDLE, SystemMode::SAFE}},
      {SystemMode::EMERGENCY, {}},
  };

  if (!load_model_card_flags()) {
    RCLCPP_WARN(get_logger(), "failed to load model card from %s", model_card_path_.c_str());
  }

  mode_req_srv_ = create_service<muto_msgs::srv::ModeRequest>(
      "/mode_request",
      std::bind(
          &ModeManagerNode::on_mode_request,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
  mode_pub_ = create_publisher<std_msgs::msg::String>("/system_mode", rclcpp::QoS(1).best_effort().durability_volatile());

  current_mode_ = SystemMode::IDLE;
  publish_mode();
}

// Verifie la presence de la transition dans la matrice d'etats.
bool ModeManagerNode::can_transition(SystemMode from, SystemMode to) const {
  auto it = transitions_.find(from);
  if (it == transitions_.end()) {
    return false;
  }
  return it->second.find(to) != it->second.end();
}

// Lecture defensive du model_card sans dependance JSON externe (scan de cles booleennes).
bool ModeManagerNode::load_model_card_flags() {
  std::ifstream in(model_card_path_);
  if (!in.good()) {
    return false;
  }
  const std::string content((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());

  auto has_true = [&](const std::string& key) {
    const std::string needle = "\"" + key + "\"";
    const size_t kpos = content.find(needle);
    if (kpos == std::string::npos) {
      return false;
    }
    const size_t tpos = content.find("true", kpos);
    const size_t fpos = content.find("false", kpos);
    return tpos != std::string::npos && (fpos == std::string::npos || tpos < fpos);
  };

  dry_run_validated_ = has_true("dry_run_validated");
  sim_real_validated_ = has_true("sim_real_validated");
  sim_real_cross_corr_validated_ = has_true("sim_real_cross_corr_validated");
  return true;
}

// Garde-fou avant activation RL en reel: les 3 validations doivent etre vraies.
bool ModeManagerNode::can_enter_rl_active(std::string& reason) const {
  if (!dry_run_validated_) {
    reason = "dry_run_validated is false";
    return false;
  }
  if (!sim_real_validated_) {
    reason = "sim_real_validated is false";
    return false;
  }
  if (!sim_real_cross_corr_validated_) {
    reason = "sim_real_cross_corr_validated is false";
    return false;
  }
  reason = "ok";
  return true;
}

// Parsing strict de la representation texte exposee aux operateurs.
bool ModeManagerNode::parse_mode(const std::string& value, SystemMode& mode_out) const {
  if (value == "IDLE") {
    mode_out = SystemMode::IDLE;
    return true;
  }
  if (value == "DRY_RUN") {
    mode_out = SystemMode::DRY_RUN;
    return true;
  }
  if (value == "RL_ACTIVE") {
    mode_out = SystemMode::RL_ACTIVE;
    return true;
  }
  if (value == "SAFE") {
    mode_out = SystemMode::SAFE;
    return true;
  }
  if (value == "MANUAL") {
    mode_out = SystemMode::MANUAL;
    return true;
  }
  if (value == "EMERGENCY") {
    mode_out = SystemMode::EMERGENCY;
    return true;
  }
  return false;
}

// Point d'entree service: parse -> verifications -> application de transition.
void ModeManagerNode::on_mode_request(
    const std::shared_ptr<muto_msgs::srv::ModeRequest::Request> request,
    std::shared_ptr<muto_msgs::srv::ModeRequest::Response> response) {
  SystemMode requested = current_mode_;
  if (!parse_mode(request->requested_mode, requested)) {
    response->success = false;
    response->message = "unknown target mode: " + request->requested_mode;
    response->current_mode = to_string(current_mode_);
    return;
  }

  std::string reason;
  if (requested == SystemMode::RL_ACTIVE && !can_enter_rl_active(reason)) {
    response->success = false;
    response->message = "RL_ACTIVE denied: " + reason;
    response->current_mode = to_string(current_mode_);
    return;
  }

  if (can_transition(current_mode_, requested)) {
    current_mode_ = requested;
    publish_mode();
    response->success = true;
    response->message = "transition applied";
    response->current_mode = to_string(current_mode_);
  } else {
    response->success = false;
    response->message = "invalid transition " + to_string(current_mode_) + " -> " + to_string(requested);
    response->current_mode = to_string(current_mode_);
  }
}

// Publication atomique de l'etat courant vers le reste du systeme.
void ModeManagerNode::publish_mode() {
  std_msgs::msg::String msg;
  msg.data = to_string(current_mode_);
  mode_pub_->publish(msg);
}

// Mapping enum -> texte stable pour logs, topics et service response.
std::string ModeManagerNode::to_string(SystemMode mode) const {
  switch (mode) {
    case SystemMode::INIT:
      return "INIT";
    case SystemMode::IDLE:
      return "IDLE";
    case SystemMode::DRY_RUN:
      return "DRY_RUN";
    case SystemMode::RL_ACTIVE:
      return "RL_ACTIVE";
    case SystemMode::SAFE:
      return "SAFE";
    case SystemMode::MANUAL:
      return "MANUAL";
    case SystemMode::EMERGENCY:
      return "EMERGENCY";
  }
  return "UNKNOWN";
}

}  // namespace muto_hardware

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<muto_hardware::ModeManagerNode>());
  rclcpp::shutdown();
  return 0;
}

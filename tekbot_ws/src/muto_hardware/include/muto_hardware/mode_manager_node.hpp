/*
 * Documentation FR: src/muto_hardware/include/muto_hardware/mode_manager_node.hpp
 * Role: composant C++ du pipeline MUTO RS avec exigences de latence et surete.
 * Details: ce fichier implemente des fonctions critiques pour la perception, le controle ou le hardware.
 * Contraintes: eviter les regressions de timing, conserver les unites physiques et la coherence des interfaces.
 * Maintenance: documenter toute hypothese metier (seuils, conversions, heuristiques) lors des modifications.
 */

/**
 * @file mode_manager_node.hpp
 * @brief Gestionnaire d'etat machine via service /mode_request.
 *
 * Le mode manager formalise les transitions autorisees et impose les preconditions
 * de securite avant RL_ACTIVE (flags model_card).
 */
#pragma once

#include <map>
#include <set>
#include <string>

#include "muto_msgs/srv/mode_request.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace muto_hardware {

enum class SystemMode {
  INIT,
  IDLE,
  DRY_RUN,
  RL_ACTIVE,
  SAFE,
  MANUAL,
  EMERGENCY,
};

/**
 * @class ModeManagerNode
 * @brief Orchestre les transitions de mode et expose une API service explicite.
 */
class ModeManagerNode : public rclcpp::Node {
public:
  /** @brief Construit la matrice de transitions et initialise le service /mode_request. */
  ModeManagerNode();

private:
  /** @brief Verifie qu'une transition from->to est presente dans la matrice. */
  bool can_transition(SystemMode from, SystemMode to) const;
  /** @brief Verifie les trois drapeaux de validation avant autorisation RL_ACTIVE. */
  bool can_enter_rl_active(std::string& reason) const;
  /** @brief Charge les flags de validation depuis le fichier model_card JSON. */
  bool load_model_card_flags();
  /** @brief Convertit une chaine de mode en enum interne. */
  bool parse_mode(const std::string& value, SystemMode& mode_out) const;
  /** @brief Callback service principal de demande de transition. */
  void on_mode_request(
      const std::shared_ptr<muto_msgs::srv::ModeRequest::Request> request,
      std::shared_ptr<muto_msgs::srv::ModeRequest::Response> response);
  /** @brief Publie le mode courant sur /system_mode. */
  void publish_mode();
  /** @brief Retourne la representation texte stable d'un mode. */
  std::string to_string(SystemMode mode) const;
  std::string model_card_path_;  ///< Chemin absolu du JSON de validation modele.
  bool dry_run_validated_{false};  ///< Flag phase dry-run validee.
  bool sim_real_validated_{false};  ///< Flag KS sim->real valide.
  bool sim_real_cross_corr_validated_{false};  ///< Flag cross-correlation valide.

  SystemMode current_mode_{SystemMode::INIT};
  std::map<SystemMode, std::set<SystemMode>> transitions_;

  rclcpp::Service<muto_msgs::srv::ModeRequest>::SharedPtr mode_req_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr mode_pub_;
};

}  // namespace muto_hardware

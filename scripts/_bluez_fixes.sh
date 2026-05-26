# shellcheck shell=bash
# Library de fixes do BlueZ para o controle PS4 (DualShock 4) parear e ficar
# conectado no Linux. Sourceada por scripts/setup_headless.sh e pair-ps4.sh.
#
# As 4 pegadinhas que cobrimos aqui (PLANO_HEADLESS_2026-05-22 §4.2):
#   1) ERTM: PS4 não fala ERTM; precisa desabilitar via modprobe option.
#   2) ControllerMode=bredr: PS4 é só clássico, BLE quebra.
#   3) AutoEnable=true: adaptador acorda "Powered: yes" no boot (headless).
#   4) ClassicBondedOnly=false: bluetoothd aceita HID sem bonding completo.
#
# Mais runtime helpers (ERTM via sysfs, rfkill unblock).
#
# Uso típico:
#     source "$REPO_DIR/scripts/_bluez_fixes.sh"
#     bluez_apply_persistent_fixes
#     if [ "$BLUEZ_NEED_RESTART" = 1 ]; then sudo systemctl restart bluetooth; fi
#     bluez_unblock_rfkill

# Aplica os 4 fixes persistentes (arquivos em /etc).
# Sets:
#   BLUEZ_NEED_RESTART=1   se algum arquivo em /etc/bluetooth/ mudou
#   BLUEZ_NEED_REBOOT=1    se o config persistente do ERTM foi escrito agora
#                          (kernel já carregou bluetooth.ko com ERTM ligado;
#                           sysfs pode aceitar o flip, senão reboot)
bluez_apply_persistent_fixes() {
    BLUEZ_NEED_RESTART=0
    BLUEZ_NEED_REBOOT=0

    # Fix 1: ERTM via /etc/modprobe.d/.
    local ertm_conf=/etc/modprobe.d/bluetooth-disable-ertm.conf
    if [ ! -f "$ertm_conf" ]; then
        echo "  → Desligando ERTM no kernel (persistente em $ertm_conf)"
        echo 'options bluetooth disable_ertm=Y' | sudo tee "$ertm_conf" >/dev/null
        BLUEZ_NEED_REBOOT=1
    fi

    # Fix 2 + 3: /etc/bluetooth/main.conf (ControllerMode=bredr, AutoEnable=true).
    local main_conf=/etc/bluetooth/main.conf
    if [ -f "$main_conf" ]; then
        if ! grep -qE '^[[:space:]]*ControllerMode[[:space:]]*=[[:space:]]*bredr' "$main_conf"; then
            echo "  → Forçando ControllerMode = bredr em $main_conf"
            sudo cp "$main_conf" "$main_conf.bak.bluez_fixes"
            if grep -qE '^[[:space:]]*#?[[:space:]]*ControllerMode' "$main_conf"; then
                sudo sed -i 's|^[[:space:]]*#\?[[:space:]]*ControllerMode[[:space:]]*=.*|ControllerMode = bredr|' "$main_conf"
            elif grep -q '^\[General\]' "$main_conf"; then
                sudo sed -i '/^\[General\]/a ControllerMode = bredr' "$main_conf"
            else
                printf '\n[General]\nControllerMode = bredr\n' | sudo tee -a "$main_conf" >/dev/null
            fi
            BLUEZ_NEED_RESTART=1
        fi

        if ! grep -qE '^[[:space:]]*AutoEnable[[:space:]]*=[[:space:]]*true' "$main_conf"; then
            echo "  → Habilitando AutoEnable = true em $main_conf"
            if grep -qE '^[[:space:]]*#?[[:space:]]*AutoEnable' "$main_conf"; then
                sudo sed -i 's|^[[:space:]]*#\?[[:space:]]*AutoEnable[[:space:]]*=.*|AutoEnable = true|' "$main_conf"
            elif grep -q '^\[Policy\]' "$main_conf"; then
                sudo sed -i '/^\[Policy\]/a AutoEnable = true' "$main_conf"
            else
                printf '\n[Policy]\nAutoEnable = true\n' | sudo tee -a "$main_conf" >/dev/null
            fi
            BLUEZ_NEED_RESTART=1
        fi
    fi

    # Fix 4: ClassicBondedOnly=false em /etc/bluetooth/input.conf.
    # Sem isso, bluetoothd recusa HID de devices sem bonding completo: PS4
    # conecta em BT mas /dev/input/jsN nunca aparece.
    local input_conf=/etc/bluetooth/input.conf
    if [ -f "$input_conf" ] && ! grep -qE '^[[:space:]]*ClassicBondedOnly[[:space:]]*=[[:space:]]*false' "$input_conf"; then
        echo "  → Setando ClassicBondedOnly = false em $input_conf"
        if grep -qE '^[[:space:]]*#?[[:space:]]*ClassicBondedOnly' "$input_conf"; then
            sudo sed -i 's|^[[:space:]]*#\?[[:space:]]*ClassicBondedOnly[[:space:]]*=.*|ClassicBondedOnly = false|' "$input_conf"
        elif grep -q '^\[General\]' "$input_conf"; then
            sudo sed -i '/^\[General\]/a ClassicBondedOnly = false' "$input_conf"
        else
            printf '\n[General]\nClassicBondedOnly = false\n' | sudo tee -a "$input_conf" >/dev/null
        fi
        BLUEZ_NEED_RESTART=1
    fi
}

# Tenta aplicar ERTM em runtime via sysfs. Retorna 0 se ficou em Y, 1 caso
# contrário (caller decide se exige reboot ou só avisa).
bluez_apply_ertm_runtime() {
    local cur
    cur="$(cat /sys/module/bluetooth/parameters/disable_ertm 2>/dev/null || echo N)"
    if [ "$cur" = "Y" ]; then return 0; fi
    echo Y | sudo tee /sys/module/bluetooth/parameters/disable_ertm >/dev/null 2>&1 || true
    cur="$(cat /sys/module/bluetooth/parameters/disable_ertm 2>/dev/null || echo N)"
    [ "$cur" = "Y" ]
}

# Destrava bluetooth no rfkill (runtime) e tenta garantir persistência via
# systemd-rfkill (que salva o estado em /var/lib/systemd/rfkill/ a cada
# shutdown e restaura no boot).
bluez_unblock_rfkill() {
    if rfkill list bluetooth 2>/dev/null | grep -q "Soft blocked: yes"; then
        echo "  → Destravando bluetooth no rfkill"
        sudo rfkill unblock bluetooth
    fi
    # Habilita systemd-rfkill se existir. Sem isso, alguns kernels acordam
    # com "Soft blocked: yes" depois de cada reboot (especialmente Pi).
    sudo systemctl enable systemd-rfkill.service >/dev/null 2>&1 || true
}

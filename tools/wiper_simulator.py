#!/usr/bin/env python3
"""
Simulador simple para la lógica `AdjustCurrentGain_Wiper`.

Escenarios que prueba:
- Normal (sin cambios)
- Saturación con MCP listo (write OK)
- Ausencia de señal (reseteo wiper)
- Saturación con MCP no listo (write falla) y ready más tarde

Ejecutar: `python tools/wiper_simulator.py`
"""
from pprint import pprint
import sys

# Default number of periods to simulate per scenario
DEFAULT_PERIODS = 5

TOTAL_PHASES = 3
I_MAX = 1843
I_MIN = 512
PER_NO_SIGNAL = 1
WIPER_POSITIONS = [64, 85, 102, 113, 120, 124, 126]


def calculate_gain(pos, invertido=False):
    if pos == 0:
        return float('inf')
    if invertido:
        return (128.0 - pos) / pos
    return pos / (128.0 - pos)


class FakeMCP:
    """Simula el MCP4131: tiene estado 'ready' y puede volverse ready tras N ticks."""
    def __init__(self, name, initial_ready=True, ready_after=0):
        self.name = name
        self.ready = initial_ready
        self.ready_after = ready_after
        self.last_written = None

    def is_ready(self):
        return self.ready

    def write(self, value):
        if self.is_ready():
            self.last_written = value
            return True
        return False

    def tick(self):
        if not self.ready and self.ready_after > 0:
            self.ready_after -= 1
            if self.ready_after == 0:
                self.ready = True


def adjust_current_gain_wiper(i_max, i_min, state, mcps):
    """
    Reproduce la lógica de `AdjustCurrentGain_Wiper` y devuelve una lista de escrituras intentadas.
    `mcps` debe ser dict con claves 'hpot3','hpot4','hpot5'.
    """
    writes = []
    for phase in range(TOTAL_PHASES):
        state['cambio_wiper'][phase] = 0

        # Saturación -> reducir ganancia
        if i_max[phase] > I_MAX or i_min[phase] < -I_MAX:
            state['cambio_wiper'][phase] = 1
            if state['wiper'][phase] > 0:
                state['wiper'][phase] -= 1
            else:
                state['wiper'][phase] = 0
                state['cambio_wiper'][phase] = 0
            state['count_noSignal'][phase] = 0

        # Ausencia de señal -> forzar ganancia mínima (wiper=0)
        elif i_max[phase] < I_MIN and i_min[phase] > -I_MIN:
            state['count_noSignal'][phase] += 1
            if state['count_noSignal'][phase] >= PER_NO_SIGNAL:
                if state['wiper'][phase] != 0:
                    state['cambio_wiper'][phase] = 1
                    state['wiper'][phase] = 0
                else:
                    state['cambio_wiper'][phase] = 0
                state['count_noSignal'][phase] = 0
            else:
                state['cambio_wiper'][phase] = 0
        else:
            state['count_noSignal'][phase] = 0

        # Si hay cambio de wiper, intentar escribir al MCP
        if state['cambio_wiper'][phase]:
            state['count_cambio_wiper'][phase] = 1

            # mapeo de hardware en el código original: case 1->hpot3, case 2->hpot4, case 0->hpot5
            if phase == 1:
                pot = mcps['hpot3']
            elif phase == 2:
                pot = mcps['hpot4']
            else:
                pot = mcps['hpot5']

            write_ok = False
            if pot.is_ready():
                write_ok = pot.write(WIPER_POSITIONS[state['wiper'][phase]])
                if write_ok:
                    state['gain_table'][phase] = calculate_gain(WIPER_POSITIONS[state['wiper'][phase]], False)

            writes.append({
                'phase': phase,
                'wiper': state['wiper'][phase],
                'attempted': True,
                'write_ok': write_ok,
                'pot_last_written': pot.last_written,
                'pot_ready': pot.is_ready(),
            })

    return writes


def init_state():
    return {
        'wiper': [6, 6, 6],
        'cambio_wiper': [0, 0, 0],
        'count_cambio_wiper': [0, 0, 0],
        'count_noSignal': [0, 0, 0],
        'gain_table': [1.0, 1.0, 1.0],
    }


def print_state(period, state, writes, mcps):
    print(f"--- Periodo {period} ---")
    print("wiper:", state['wiper'])
    print("gain_table:", [round(g,4) if g != float('inf') else 'inf' for g in state['gain_table']])
    print("cambio_wiper:", state['cambio_wiper'])
    print("count_noSignal:", state['count_noSignal'])
    for w in writes:
        print(f" phase {w['phase']}: attempted write -> ok={w['write_ok']}, pot_ready={w['pot_ready']}, pot_last_written={w['pot_last_written']}")
    print("mcps last_written:", {k: v.last_written for k, v in mcps.items()})
    print()


def scenario_normal(n_periods=DEFAULT_PERIODS):
    print("Scenario: Normal (sin cambios)")
    state = init_state()
    mcps = { 'hpot3': FakeMCP('hpot3'), 'hpot4': FakeMCP('hpot4'), 'hpot5': FakeMCP('hpot5') }
    for p in range(1, n_periods + 1):
        i_max = [1000, 1000, 1000]
        i_min = [-1000, -1000, -1000]
        writes = adjust_current_gain_wiper(i_max, i_min, state, mcps)
        for pot in mcps.values():
            pot.tick()
        print_state(p, state, writes, mcps)


def scenario_saturation_ok(n_periods=DEFAULT_PERIODS):
    print("Scenario: Saturación en fase 0, MCP listo (write OK)")
    state = init_state()
    mcps = { 'hpot3': FakeMCP('hpot3', True), 'hpot4': FakeMCP('hpot4', True), 'hpot5': FakeMCP('hpot5', True) }
    for p in range(1, n_periods + 1):
        # Mantener saturación en la fase 0 durante los periodos
        i_max = [1900, 1000, 1000]
        i_min = [-1000, -1000, -1000]
        writes = adjust_current_gain_wiper(i_max, i_min, state, mcps)
        for pot in mcps.values():
            pot.tick()
        print_state(p, state, writes, mcps)


def scenario_no_signal(n_periods=DEFAULT_PERIODS):
    print("Scenario: Ausencia de señal en fase 1 -> forzar wiper=0")
    state = init_state()
    mcps = { 'hpot3': FakeMCP('hpot3', True), 'hpot4': FakeMCP('hpot4', True), 'hpot5': FakeMCP('hpot5', True) }
    for p in range(1, n_periods + 1):
        i_max = [1000, 400, 1000]   # fase1 por debajo de I_MIN
        i_min = [-1000, 0, -1000]
        writes = adjust_current_gain_wiper(i_max, i_min, state, mcps)
        for pot in mcps.values():
            pot.tick()
        print_state(p, state, writes, mcps)


def scenario_spi_not_ready_then_ready(n_periods=DEFAULT_PERIODS):
    print("Scenario: Saturación en fase 0 pero MCP (hpot5) no listo, listo tras 2 periodos")
    state = init_state()
    # hpot5 no listo inicialmente, se volverá ready tras 2 ticks
    mcps = { 'hpot3': FakeMCP('hpot3', True), 'hpot4': FakeMCP('hpot4', True), 'hpot5': FakeMCP('hpot5', False, ready_after=2) }

    for p in range(1, n_periods + 1):
        i_max = [1900, 1000, 1000]
        i_min = [-1000, -1000, -1000]
        writes = adjust_current_gain_wiper(i_max, i_min, state, mcps)
        for pot in mcps.values():
            pot.tick()
        print_state(p, state, writes, mcps)


def main():
    SCENARIOS = {
        'normal': scenario_normal,
        'sat_ok': scenario_saturation_ok,
        'no_signal': scenario_no_signal,
        'spi_delayed': scenario_spi_not_ready_then_ready,
    }

    if len(sys.argv) > 1:
        name = sys.argv[1]
        try:
            n = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_PERIODS
        except ValueError:
            print('El segundo argumento debe ser un entero (nº periodos).')
            return

        if name == 'all':
            for fn in SCENARIOS.values():
                fn(n)
        else:
            fn = SCENARIOS.get(name)
            if not fn:
                print('Opciones:', ', '.join(list(SCENARIOS.keys()) + ['all']))
                return
            fn(n)
    else:
        # Por defecto ejecutar todos con DEFAULT_PERIODS
        for fn in SCENARIOS.values():
            fn(DEFAULT_PERIODS)


if __name__ == '__main__':
    main()

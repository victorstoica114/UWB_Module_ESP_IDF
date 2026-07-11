#!/usr/bin/env python3
"""Dump BQ25792 registers exposed by the module /status endpoint."""

from __future__ import annotations

import argparse
import json
import sys
import urllib.request
from pathlib import Path


REG_NAMES = {
    0x00: "Minimal_System_Voltage",
    0x01: "Charge_Voltage_Limit_MSB",
    0x02: "Charge_Voltage_Limit_LSB",
    0x03: "Charge_Current_Limit_MSB",
    0x04: "Charge_Current_Limit_LSB",
    0x05: "Input_Voltage_Limit",
    0x06: "Input_Current_Limit_MSB",
    0x07: "Input_Current_Limit_LSB",
    0x08: "Precharge_Control",
    0x09: "Termination_Control",
    0x0A: "Recharge_Control",
    0x0B: "VOTG_Regulation_MSB",
    0x0C: "VOTG_Regulation_LSB",
    0x0D: "IOTG_Regulation",
    0x0E: "Timer_Control",
    0x0F: "Charger_Control_0",
    0x10: "Charger_Control_1",
    0x11: "Charger_Control_2",
    0x12: "Charger_Control_3",
    0x13: "Charger_Control_4",
    0x14: "Charger_Control_5",
    0x15: "Reserved",
    0x16: "Temperature_Control",
    0x17: "NTC_Control_0",
    0x18: "NTC_Control_1",
    0x19: "ICO_Current_Limit_MSB",
    0x1A: "ICO_Current_Limit_LSB",
    0x1B: "Charger_Status_0",
    0x1C: "Charger_Status_1",
    0x1D: "Charger_Status_2",
    0x1E: "Charger_Status_3",
    0x1F: "Charger_Status_4",
    0x20: "Fault_Status_0",
    0x21: "Fault_Status_1",
    0x22: "Charger_Flag_0",
    0x23: "Charger_Flag_1",
    0x24: "Charger_Flag_2",
    0x25: "Charger_Flag_3",
    0x26: "Fault_Flag_0",
    0x27: "Fault_Flag_1",
    0x28: "Charger_Mask_0",
    0x29: "Charger_Mask_1",
    0x2A: "Charger_Mask_2",
    0x2B: "Charger_Mask_3",
    0x2C: "Fault_Mask_0",
    0x2D: "Fault_Mask_1",
    0x2E: "ADC_Control",
    0x2F: "ADC_Function_Disable_0",
    0x30: "ADC_Function_Disable_1",
    0x31: "IBUS_ADC_MSB",
    0x32: "IBUS_ADC_LSB",
    0x33: "IBAT_ADC_MSB",
    0x34: "IBAT_ADC_LSB",
    0x35: "VBUS_ADC_MSB",
    0x36: "VBUS_ADC_LSB",
    0x37: "VAC1_ADC_MSB",
    0x38: "VAC1_ADC_LSB",
    0x39: "VAC2_ADC_MSB",
    0x3A: "VAC2_ADC_LSB",
    0x3B: "VBAT_ADC_MSB",
    0x3C: "VBAT_ADC_LSB",
    0x3D: "VSYS_ADC_MSB",
    0x3E: "VSYS_ADC_LSB",
    0x3F: "TS_ADC_MSB",
    0x40: "TS_ADC_LSB",
    0x41: "TDIE_ADC_MSB",
    0x42: "TDIE_ADC_LSB",
    0x43: "DPLUS_ADC_MSB",
    0x44: "DPLUS_ADC_LSB",
    0x45: "DMINUS_ADC_MSB",
    0x46: "DMINUS_ADC_LSB",
    0x47: "DPDM_Driver",
    0x48: "Part_Information",
}


def load_targets(args: argparse.Namespace) -> list[str]:
    targets: list[str] = []
    targets.extend(args.hosts or [])
    if args.target_list:
        for line in Path(args.target_list).read_text(encoding="utf-8").splitlines():
            clean = line.split("#", 1)[0].strip()
            if clean:
                targets.append(clean)
    return targets


def fetch_status(host: str, timeout: float) -> dict:
    with urllib.request.urlopen(f"http://{host}/status", timeout=timeout) as response:
        return json.loads(response.read().decode("utf-8"))


def raw_bytes(status: dict) -> bytes:
    raw_hex = status.get("charger_raw_hex") or ""
    if len(raw_hex) % 2:
        raise ValueError("charger_raw_hex has odd length")
    return bytes.fromhex(raw_hex)


def print_summary(host: str, status: dict, raw: bytes) -> None:
    print(f"\n{host} {status.get('hostname', '')}")
    print(
        "present={present} read_ok={read_ok} err={err} "
        "part={part} pn={pn} rev={rev} adc={adc} sample={sample} "
        "wd={watchdog} writes={writes}".format(
            present=status.get("charger_present"),
            read_ok=status.get("charger_read_ok"),
            err=status.get("charger_last_error_name"),
            part=status.get("charger_part_info"),
            pn=status.get("charger_part_number"),
            rev=status.get("charger_device_revision"),
            adc=status.get("charger_adc_enabled"),
            sample=status.get("charger_adc_sample"),
            watchdog=status.get("charger_watchdog_setting"),
            writes="enabled"
            if status.get("charger_config_writes_enabled")
            else "disabled",
        )
    )
    print(
        "limits: VSYSMIN={vsysmin}mV VREG={vreg}mV ICHG={ichg}mA "
        "VINDPM={vindpm}mV IINDPM={iindpm}mA".format(
            vsysmin=status.get("charger_minimal_system_voltage_mv"),
            vreg=status.get("charger_charge_voltage_limit_mv"),
            ichg=status.get("charger_charge_current_limit_ma"),
            vindpm=status.get("charger_input_voltage_limit_mv"),
            iindpm=status.get("charger_input_current_limit_ma"),
        )
    )
    print(
        "VBAT={vbat}mV VSYS={vsys}mV VBUS={vbus}mV "
        "IBUS={ibus}mA IBAT={ibat}mA TDIE={tdie}C".format(
            vbat=status.get("charger_vbat_mv"),
            vsys=status.get("charger_vsys_mv"),
            vbus=status.get("charger_vbus_mv"),
            ibus=status.get("charger_ibus_ma"),
            ibat=status.get("charger_ibat_ma"),
            tdie=status.get("charger_tdie_c"),
        )
    )
    print(
        "pins: BOARD_PG={pg} asserted={pg_asserted} PG_STAT={pg_stat} INT={int_level} "
        "irq={irq} irq_age={irq_age}ms QON_CMD={qon} asserted={qon_asserted}".format(
            pg=status.get("charger_pg_gpio_level"),
            pg_asserted=status.get("charger_pg_asserted"),
            pg_stat=status.get("charger_pg_stat"),
            int_level=status.get("charger_int_gpio_level"),
            irq=status.get("charger_int_irq_count"),
            irq_age=status.get("charger_int_last_irq_age_ms"),
            qon=status.get("charger_qon_gpio_level"),
            qon_asserted=status.get("charger_qon_asserted"),
        )
    )
    print(
        "last write: count={count} errors={errors} reg={reg} before={before} "
        "after={after} err={err} age={age}ms".format(
            count=status.get("charger_write_count"),
            errors=status.get("charger_write_error_count"),
            reg=status.get("charger_last_write_reg"),
            before=status.get("charger_last_write_before"),
            after=status.get("charger_last_write_after"),
            err=status.get("charger_last_write_error_name"),
            age=status.get("charger_last_write_age_ms"),
        )
    )
    print("reg  value  name")
    for reg, value in enumerate(raw):
        print(f"0x{reg:02X}  0x{value:02X}   {REG_NAMES.get(reg, '')}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("hosts", nargs="*", help="Module IP/hostnames")
    parser.add_argument("--target-list", default="", help="File with one host per line")
    parser.add_argument("--timeout", type=float, default=4.0)
    args = parser.parse_args()

    targets = load_targets(args)
    if not targets:
        parser.error("provide at least one host or --target-list")

    ok = True
    for host in targets:
        try:
            status = fetch_status(host, args.timeout)
            raw = raw_bytes(status)
            print_summary(host, status, raw)
        except Exception as exc:  # noqa: BLE001 - diagnostic CLI should keep going
            ok = False
            print(f"\n{host} ERROR: {exc}", file=sys.stderr)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())

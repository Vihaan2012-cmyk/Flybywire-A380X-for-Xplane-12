const STRUCT_UNITS = new Set(['latlonalt', 'latlonaltpbh', 'pbh', 'pid_struct', 'xyz']);

let installed = false;

export function installVarSetter(): void {
  if (installed || typeof SimVar === 'undefined' || typeof Coherent === 'undefined') {
    return;
  }
  installed = true;

  const original = SimVar.SetSimVarValue;
  SimVar.SetSimVarValue = function (name: string, unit: string, value: any, dataSource = ''): Promise<void> {
    const normalizedUnit = String(unit).toLowerCase();
    if (value === undefined || value === null || STRUCT_UNITS.has(normalizedUnit)) {
      return original.call(SimVar, name, unit, value, dataSource);
    }
    const id = SimVar.GetRegisteredId(name, unit, dataSource);
    switch (normalizedUnit) {
      case 'string':
        return Coherent.call('setValueReg_String', id, value);
      case 'bool':
      case 'boolean':
        return Coherent.call('setValueReg_Bool', id, !!value);
      default:
        return Coherent.call('setValueReg_Number', id, value);
    }
  };
}

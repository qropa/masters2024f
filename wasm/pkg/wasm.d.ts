/* tslint:disable */
/* eslint-disable */
/**
* @param {string} input
* @param {string} output
* @param {number} t
* @returns {Res}
*/
export function vis(input: string, output: string, t: number): Res;
/**
* @param {string} input
* @param {string} output
* @returns {number}
*/
export function get_max_turn(input: string, output: string): number;
/**
*/
export class Res {
  free(): void;
/**
*/
  error: string;
/**
*/
  score: number;
/**
*/
  svg: string;
}

export type InitInput = RequestInfo | URL | Response | BufferSource | WebAssembly.Module;

export interface InitOutput {
  readonly memory: WebAssembly.Memory;
  readonly __wbg_res_free: (a: number) => void;
  readonly __wbg_get_res_score: (a: number) => number;
  readonly __wbg_set_res_score: (a: number, b: number) => void;
  readonly __wbg_get_res_error: (a: number, b: number) => void;
  readonly __wbg_set_res_error: (a: number, b: number, c: number) => void;
  readonly __wbg_get_res_svg: (a: number, b: number) => void;
  readonly __wbg_set_res_svg: (a: number, b: number, c: number) => void;
  readonly vis: (a: number, b: number, c: number, d: number, e: number) => number;
  readonly get_max_turn: (a: number, b: number, c: number, d: number) => number;
  readonly __wbindgen_add_to_stack_pointer: (a: number) => number;
  readonly __wbindgen_free: (a: number, b: number, c: number) => void;
  readonly __wbindgen_malloc: (a: number, b: number) => number;
  readonly __wbindgen_realloc: (a: number, b: number, c: number, d: number) => number;
}

export type SyncInitInput = BufferSource | WebAssembly.Module;
/**
* Instantiates the given `module`, which can either be bytes or
* a precompiled `WebAssembly.Module`.
*
* @param {SyncInitInput} module
*
* @returns {InitOutput}
*/
export function initSync(module: SyncInitInput): InitOutput;

/**
* If `module_or_path` is {RequestInfo} or {URL}, makes a request and
* for everything else, calls `WebAssembly.instantiate` directly.
*
* @param {InitInput | Promise<InitInput>} module_or_path
*
* @returns {Promise<InitOutput>}
*/
export default function __wbg_init (module_or_path?: InitInput | Promise<InitInput>): Promise<InitOutput>;

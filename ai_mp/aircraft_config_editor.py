#!/usr/bin/env python3
"""
飞机参数编辑器 - 固定翼飞机物理参数配置工具
用于调整 SITL 仿真飞机参数，保存自定义配置
"""
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import json
import os

# ===== 默认参数 (ArduPilot SITL Plane) =====
DEFAULT_PARAMS = {
    "_info": {
        "name": "ArduPilot SITL Default",
        "description": "基于 Skywalker 2013 的默认模型",
    },
    "physics": {
        "mass": {"value": 2.0, "unit": "kg", "label": "整机质量", "min": 0.1, "max": 100, "step": 0.1},
        "wing_area": {"value": 0.45, "unit": "m²", "label": "翼面积", "min": 0.01, "max": 10, "step": 0.01},
        "wingspan": {"value": 1.88, "unit": "m", "label": "翼展", "min": 0.1, "max": 20, "step": 0.01},
        "chord": {"value": 0.24, "unit": "m", "label": "平均弦长", "min": 0.01, "max": 5, "step": 0.01},
        "oswald": {"value": 0.9, "unit": "", "label": "Oswald效率因子", "min": 0.5, "max": 1.0, "step": 0.01},
        "alpha_stall": {"value": 27.0, "unit": "°", "label": "失速迎角", "min": 5, "max": 45, "step": 0.5},
    },
    "aero_lift": {
        "c_lift_0": {"value": 0.56, "unit": "", "label": "零迎角升力系数 C_L0", "min": 0, "max": 2, "step": 0.01},
        "c_lift_a": {"value": 6.9, "unit": "/rad", "label": "升力线斜率 C_Lα", "min": 1, "max": 15, "step": 0.1},
        "c_lift_deltae": {"value": 0.0, "unit": "", "label": "升降舵升力系数", "min": -2, "max": 2, "step": 0.01},
        "c_lift_q": {"value": 0.0, "unit": "", "label": "俯仰速率升力系数", "min": -10, "max": 10, "step": 0.1},
    },
    "aero_drag": {
        "c_drag_p": {"value": 0.1, "unit": "", "label": "寄生阻力系数 C_Dp", "min": 0.01, "max": 0.5, "step": 0.01},
        "c_drag_q": {"value": 0.0, "unit": "", "label": "速度阻力系数", "min": 0, "max": 2, "step": 0.01},
        "c_drag_deltae": {"value": 0.0, "unit": "", "label": "升降舵阻力系数", "min": 0, "max": 1, "step": 0.01},
    },
    "aero_moment": {
        "c_m_0": {"value": 0.045, "unit": "", "label": "零迎角俯仰力矩 C_M0", "min": -0.5, "max": 0.5, "step": 0.005},
        "c_m_a": {"value": -0.7, "unit": "/rad", "label": "俯仰力矩斜率 C_Mα", "min": -3, "max": 0, "step": 0.05},
        "c_m_q": {"value": -20.0, "unit": "", "label": "俯仰阻尼 C_Mq", "min": -50, "max": 0, "step": 0.5},
        "c_m_deltae": {"value": 1.0, "unit": "", "label": "升降舵力矩系数", "min": 0, "max": 5, "step": 0.1},
    },
    "aero_lateral": {
        "c_y_b": {"value": -0.98, "unit": "/rad", "label": "侧力斜率 C_Yβ", "min": -3, "max": 0, "step": 0.05},
        "c_l_b": {"value": -0.12, "unit": "/rad", "label": "滚转力矩斜率 C_lβ", "min": -1, "max": 0, "step": 0.01},
        "c_l_p": {"value": -1.0, "unit": "", "label": "滚转阻尼 C_lp", "min": -3, "max": 0, "step": 0.05},
        "c_l_r": {"value": 0.14, "unit": "", "label": "偏航诱导滚转 C_lr", "min": -1, "max": 1, "step": 0.01},
        "c_l_deltaa": {"value": 0.25, "unit": "", "label": "副翼滚转力矩", "min": 0, "max": 1, "step": 0.01},
        "c_n_b": {"value": 0.25, "unit": "/rad", "label": "偏航力矩斜率 C_nβ", "min": 0, "max": 1, "step": 0.01},
        "c_n_r": {"value": -1.0, "unit": "", "label": "偏航阻尼 C_nr", "min": -5, "max": 0, "step": 0.05},
    },
    "control_surface": {
        "deltaa_max": {"value": 20.0, "unit": "°", "label": "副翼最大偏角", "min": 5, "max": 45, "step": 1},
        "deltae_max": {"value": 20.0, "unit": "°", "label": "升降舵最大偏角", "min": 5, "max": 45, "step": 1},
        "deltar_max": {"value": 20.0, "unit": "°", "label": "方向舵最大偏角", "min": 5, "max": 45, "step": 1},
    },
    "flightEnvelope": {
        "airspeed_cruise": {"value": 22.0, "unit": "m/s", "label": "巡航空速", "min": 5, "max": 80, "step": 0.5},
        "airspeed_max": {"value": 30.0, "unit": "m/s", "label": "最大空速", "min": 10, "max": 150, "step": 1},
        "airspeed_min": {"value": 10.0, "unit": "m/s", "label": "最小空速(失速)", "min": 3, "max": 30, "step": 0.5},
        "thr_max": {"value": 100, "unit": "%", "label": "最大油门", "min": 50, "max": 100, "step": 1},
        "trim_throttle": {"value": 50, "unit": "%", "label": "巡航油门", "min": 10, "max": 80, "step": 1},
        "ptch_lim_max": {"value": 25.0, "unit": "°", "label": "最大俯仰角", "min": 10, "max": 45, "step": 1},
        "ptch_lim_min": {"value": -20.0, "unit": "°", "label": "最小俯仰角", "min": -45, "max": -5, "step": 1},
        "roll_limit": {"value": 65.0, "unit": "°", "label": "最大滚转角", "min": 20, "max": 90, "step": 1},
    },
    "pid_roll": {
        "rll_rate_p": {"value": 0.3, "unit": "", "label": "滚转角速度 P", "min": 0.01, "max": 2, "step": 0.01},
        "rll_rate_i": {"value": 0.25, "unit": "", "label": "滚转角速度 I", "min": 0, "max": 2, "step": 0.01},
        "rll_rate_d": {"value": 0.017, "unit": "", "label": "滚转角速度 D", "min": 0, "max": 0.5, "step": 0.001},
        "rll_rate_ff": {"value": 0.237, "unit": "", "label": "滚转角速度 FF", "min": 0, "max": 1, "step": 0.01},
    },
    "pid_pitch": {
        "ptch_rate_p": {"value": 0.15, "unit": "", "label": "俯仰角速度 P", "min": 0.01, "max": 2, "step": 0.01},
        "ptch_rate_i": {"value": 0.11, "unit": "", "label": "俯仰角速度 I", "min": 0, "max": 2, "step": 0.01},
        "ptch_rate_d": {"value": 0.007, "unit": "", "label": "俯仰角速度 D", "min": 0, "max": 0.5, "step": 0.001},
        "ptch_rate_ff": {"value": 0.596, "unit": "", "label": "俯仰角速度 FF", "min": 0, "max": 2, "step": 0.01},
    },
}

CATEGORY_LABELS = {
    "_info": "基本信息",
    "physics": "物理特征",
    "aero_lift": "气动 - 升力",
    "aero_drag": "气动 - 阻力",
    "aero_moment": "气动 - 力矩",
    "aero_lateral": "气动 - 横侧",
    "control_surface": "舵面限制",
    "flightEnvelope": "飞行包线",
    "pid_roll": "PID - 滚转",
    "pid_pitch": "PID - 俯仰",
}

PRESETS = {
    "SITL Default (Skywalker 2013)": DEFAULT_PARAMS,
    "小型教练机 (1.2m翼展)": {
        "_info": {"name": "小型教练机", "description": "1.2m翼展轻型练习机"},
        "physics": {
            "mass": {"value": 0.8, "unit": "kg", "label": "整机质量", "min": 0.1, "max": 100, "step": 0.1},
            "wing_area": {"value": 0.25, "unit": "m²", "label": "翼面积", "min": 0.01, "max": 10, "step": 0.01},
            "wingspan": {"value": 1.2, "unit": "m", "label": "翼展", "min": 0.1, "max": 20, "step": 0.01},
            "chord": {"value": 0.18, "unit": "m", "label": "平均弦长", "min": 0.01, "max": 5, "step": 0.01},
            "oswald": {"value": 0.85, "unit": "", "label": "Oswald效率因子", "min": 0.5, "max": 1.0, "step": 0.01},
            "alpha_stall": {"value": 18.0, "unit": "°", "label": "失速迎角", "min": 5, "max": 45, "step": 0.5},
        },
        "flightEnvelope": {
            "airspeed_cruise": {"value": 15.0, "unit": "m/s", "label": "巡航空速", "min": 5, "max": 80, "step": 0.5},
            "airspeed_max": {"value": 25.0, "unit": "m/s", "label": "最大空速", "min": 10, "max": 150, "step": 1},
            "airspeed_min": {"value": 7.0, "unit": "m/s", "label": "最小空速(失速)", "min": 3, "max": 30, "step": 0.5},
        },
    },
    "中型载荷机 (2.5m翼展)": {
        "_info": {"name": "中型载荷机", "description": "2.5m翼展载荷运输机"},
        "physics": {
            "mass": {"value": 5.0, "unit": "kg", "label": "整机质量", "min": 0.1, "max": 100, "step": 0.1},
            "wing_area": {"value": 0.9, "unit": "m²", "label": "翼面积", "min": 0.01, "max": 10, "step": 0.01},
            "wingspan": {"value": 2.5, "unit": "m", "label": "翼展", "min": 0.1, "max": 20, "step": 0.01},
            "chord": {"value": 0.36, "unit": "m", "label": "平均弦长", "min": 0.01, "max": 5, "step": 0.01},
            "oswald": {"value": 0.82, "unit": "", "label": "Oswald效率因子", "min": 0.5, "max": 1.0, "step": 0.01},
            "alpha_stall": {"value": 20.0, "unit": "°", "label": "失速迎角", "min": 5, "max": 45, "step": 0.5},
        },
        "flightEnvelope": {
            "airspeed_cruise": {"value": 20.0, "unit": "m/s", "label": "巡航空速", "min": 5, "max": 80, "step": 0.5},
            "airspeed_max": {"value": 35.0, "unit": "m/s", "label": "最大空速", "min": 10, "max": 150, "step": 1},
            "airspeed_min": {"value": 9.0, "unit": "m/s", "label": "最小空速(失速)", "min": 3, "max": 30, "step": 0.5},
        },
    },
}


class AircraftEditor:
    def __init__(self, root):
        self.root = root
        self.root.title("飞机参数编辑器 - SITL Fixed Wing Config")
        self.root.geometry("900x700")
        self.root.minsize(800, 600)

        # Load saved config or default
        self.config_path = os.path.join(os.path.dirname(__file__), "aircraft_config.json")
        self.params = self._deep_copy_params(DEFAULT_PARAMS)
        self._load_config()

        # Create UI
        self._build_menu()
        self._build_main()

    def _deep_copy_params(self, src):
        return json.loads(json.dumps(src))

    def _load_config(self):
        if os.path.exists(self.config_path):
            try:
                with open(self.config_path, "r", encoding="utf-8") as f:
                    saved = json.load(f)
                # Merge saved values into params
                for cat, params in saved.items():
                    if cat in self.params:
                        for key, val in params.items():
                            if key in self.params[cat] and "value" in self.params[cat][key]:
                                if isinstance(val, dict) and "value" in val:
                                    self.params[cat][key]["value"] = val["value"]
                                else:
                                    self.params[cat][key]["value"] = val
                self.root.title(f"飞机参数编辑器 - {saved.get('_info', {}).get('name', 'Custom')}")
            except Exception:
                pass

    def _build_menu(self):
        menubar = tk.Menu(self.root)
        self.root.config(menu=menubar)

        file_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="文件", menu=file_menu)
        file_menu.add_command(label="保存配置", command=self._save_config, accelerator="Ctrl+S")
        file_menu.add_command(label="加载配置", command=self._load_config_dialog)
        file_menu.add_separator()
        file_menu.add_command(label="导出 .parm 文件", command=self._export_parm)
        file_menu.add_command(label="导出到WSL", command=self._export_to_wsl)
        file_menu.add_separator()
        file_menu.add_command(label="退出", command=self.root.quit)

        preset_menu = tk.Menu(menubar, tearoff=0)
        menubar.add_cascade(label="预设", menu=preset_menu)
        for name in PRESETS:
            preset_menu.add_command(label=name, command=lambda n=name: self._apply_preset(n))

        self.root.bind("<Control-s>", lambda e: self._save_config())

    def _build_main(self):
        # Top bar: info
        info_frame = ttk.LabelFrame(self.root, text="基本信息", padding=5)
        info_frame.pack(fill="x", padx=5, pady=5)

        ttk.Label(info_frame, text="名称:").grid(row=0, column=0, sticky="e")
        self.name_var = tk.StringVar(value=self.params["_info"]["name"])
        ttk.Entry(info_frame, textvariable=self.name_var, width=40).grid(row=0, column=1, sticky="w", padx=5)

        ttk.Label(info_frame, text="描述:").grid(row=0, column=2, sticky="e")
        self.desc_var = tk.StringVar(value=self.params["_info"]["description"])
        ttk.Entry(info_frame, textvariable=self.desc_var, width=50).grid(row=0, column=3, sticky="w", padx=5)

        # Derived info
        self.info_label = ttk.Label(info_frame, text="", foreground="blue")
        self.info_label.grid(row=1, column=0, columnspan=4, sticky="w", pady=2)
        self._update_info()

        # Paned window for categories
        paned = ttk.PanedWindow(self.root, orient="horizontal")
        paned.pack(fill="both", expand=True, padx=5, pady=5)

        # Left: category list
        left_frame = ttk.Frame(paned, width=180)
        paned.add(left_frame, weight=0)

        ttk.Label(left_frame, text="参数分类:", font=("", 10, "bold")).pack(anchor="w", padx=5, pady=5)

        self.category_var = tk.StringVar()
        categories = [(k, CATEGORY_LABELS.get(k, k)) for k in self.params.keys() if k != "_info"]
        self.cat_buttons = {}
        for cat_key, cat_label in categories:
            btn = ttk.Radiobutton(
                left_frame, text=cat_label, variable=self.category_var, value=cat_key,
                command=lambda c=cat_key: self._show_category(c)
            )
            btn.pack(anchor="w", padx=10, pady=2)
            self.cat_buttons[cat_key] = btn

        # Right: parameter editor
        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=1)

        # Scrollable canvas for parameters
        canvas_frame = ttk.Frame(right_frame)
        canvas_frame.pack(fill="both", expand=True)

        self.canvas = tk.Canvas(canvas_frame)
        scrollbar = ttk.Scrollbar(canvas_frame, orient="vertical", command=self.canvas.yview)
        self.param_frame = ttk.Frame(self.canvas)

        self.param_frame.bind("<Configure>", lambda e: self.canvas.configure(scrollregion=self.canvas.bbox("all")))
        self.canvas.create_window((0, 0), window=self.param_frame, anchor="nw")
        self.canvas.configure(yscrollcommand=scrollbar.set)

        self.canvas.pack(side="left", fill="both", expand=True)
        scrollbar.pack(side="right", fill="y")

        # Bind mousewheel
        self.canvas.bind_all("<MouseWheel>", lambda e: self.canvas.yview_scroll(int(-1 * (e.delta / 120)), "units"))

        # Bottom: buttons
        btn_frame = ttk.Frame(self.root)
        btn_frame.pack(fill="x", padx=5, pady=5)

        ttk.Button(btn_frame, text="重置为默认", command=self._reset_defaults).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="导出 .parm", command=self._export_parm).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="导出到WSL并启动SITL", command=self._export_to_wsl).pack(side="left", padx=5)
        ttk.Button(btn_frame, text="保存配置", command=self._save_config).pack(side="right", padx=5)

        # Show first category
        self.category_var.set("physics")
        self._show_category("physics")

    def _show_category(self, cat_key):
        # Clear current params
        for widget in self.param_frame.winfo_children():
            widget.destroy()

        if cat_key not in self.params:
            return

        cat_label = CATEGORY_LABELS.get(cat_key, cat_key)
        ttk.Label(self.param_frame, text=cat_label, font=("", 12, "bold")).grid(
            row=0, column=0, columnspan=4, sticky="w", pady=(5, 10), padx=5
        )

        row = 1
        headers = ["参数", "当前值", "单位", "范围"]
        for col, h in enumerate(headers):
            ttk.Label(self.param_frame, text=h, font=("", 9, "bold")).grid(
                row=row, column=col, sticky="w", padx=10, pady=2
            )
        row += 1

        ttk.Separator(self.param_frame, orient="horizontal").grid(
            row=row, column=0, columnspan=4, sticky="ew", pady=2
        )
        row += 1

        for key, param in self.params[cat_key].items():
            if key.startswith("_"):
                continue
            label = param.get("label", key)
            unit = param.get("unit", "")
            value = param["value"]
            vmin = param.get("min", 0)
            vmax = param.get("max", 100)
            step = param.get("step", 0.1)

            ttk.Label(self.param_frame, text=label).grid(row=row, column=0, sticky="w", padx=10, pady=3)

            # Entry + slider
            var = tk.DoubleVar(value=value)
            entry = ttk.Entry(self.param_frame, textvariable=var, width=10)
            entry.grid(row=row, column=1, padx=5, pady=3)

            ttk.Label(self.param_frame, text=unit, width=6).grid(row=row, column=2, sticky="w", padx=5)

            range_text = f"[{vmin} ~ {vmax}]"
            ttk.Label(self.param_frame, text=range_text, foreground="gray").grid(
                row=row, column=3, sticky="w", padx=5
            )

            # Slider
            slider = ttk.Scale(
                self.param_frame, from_=vmin, to=vmax, variable=var, orient="horizontal",
                command=lambda v, k=key, c=cat_key, e=entry, s=step: self._on_slider(v, k, c, e, s)
            )
            slider.grid(row=row + 1, column=0, columnspan=4, sticky="ew", padx=15, pady=1)

            # Bind entry change
            entry.bind("<Return>", lambda e, k=key, c=cat_key, v=var: self._on_entry(k, c, v))
            entry.bind("<FocusOut>", lambda e, k=key, c=cat_key, v=var: self._on_entry(k, c, v))

            # Store reference
            self.params[cat_key][key]["_var"] = var

            row += 2

    def _on_slider(self, val, key, cat, entry, step):
        try:
            v = round(float(val), 4)
            self.params[cat][key]["value"] = v
            if "_var" in self.params[cat][key]:
                self.params[cat][key]["_var"].set(v)
            self._update_info()
        except Exception:
            pass

    def _on_entry(self, key, cat, var):
        try:
            v = float(var.get())
            self.params[cat][key]["value"] = v
            self._update_info()
        except ValueError:
            pass

    def _update_info(self):
        try:
            mass = self.params["physics"]["mass"]["value"]
            area = self.params["physics"]["wing_area"]["value"]
            span = self.params["physics"]["wingspan"]["value"]
            chord = self.params["physics"]["chord"]["value"]
            wingloading = mass * 9.81 / area
            ar = span ** 2 / area
            stall_speed = self.params["flightEnvelope"]["airspeed_min"]["value"]
            cruise_speed = self.params["flightEnvelope"]["airspeed_cruise"]["value"]
            info = (
                f"翼载荷: {wingloading:.1f} N/m² | "
                f"展弦比: {ar:.2f} | "
                f"巡航: {cruise_speed} m/s ({cruise_speed * 3.6:.0f} km/h) | "
                f"失速: {stall_speed} m/s ({stall_speed * 3.6:.0f} km/h)"
            )
            self.info_label.config(text=info)
        except Exception:
            pass

    def _apply_preset(self, name):
        if name in PRESETS:
            self.params = self._deep_copy_params(PRESETS[name])
            self.name_var.set(self.params["_info"]["name"])
            self.desc_var.set(self.params["_info"]["description"])
            self._show_category(self.category_var.get() or "physics")
            self._update_info()
            self.root.title(f"飞机参数编辑器 - {name}")

    def _reset_defaults(self):
        self.params = self._deep_copy_params(DEFAULT_PARAMS)
        self.name_var.set(self.params["_info"]["name"])
        self.desc_var.set(self.params["_info"]["description"])
        self._show_category(self.category_var.get() or "physics")
        self._update_info()

    def _save_config(self):
        self.params["_info"]["name"] = self.name_var.get()
        self.params["_info"]["description"] = self.desc_var.get()
        save_data = {}
        for cat, params in self.params.items():
            save_data[cat] = {}
            for key, val in params.items():
                if not key.startswith("_"):
                    save_data[cat][key] = {"value": val["value"]}
        try:
            with open(self.config_path, "w", encoding="utf-8") as f:
                json.dump(save_data, f, indent=2, ensure_ascii=False)
            messagebox.showinfo("保存成功", f"配置已保存到:\n{self.config_path}")
        except Exception as e:
            messagebox.showerror("保存失败", str(e))

    def _load_config_dialog(self):
        path = filedialog.askopenfilename(
            filetypes=[("JSON", "*.json"), ("所有文件", "*.*")]
        )
        if path:
            try:
                with open(path, "r", encoding="utf-8") as f:
                    saved = json.load(f)
                for cat, params in saved.items():
                    if cat in self.params:
                        for key, val in params.items():
                            if key in self.params[cat] and "value" in self.params[cat][key]:
                                if isinstance(val, dict) and "value" in val:
                                    self.params[cat][key]["value"] = val["value"]
                self._show_category(self.category_var.get() or "physics")
                self._update_info()
            except Exception as e:
                messagebox.showerror("加载失败", str(e))

    def _export_parm(self):
        path = filedialog.asksaveasfilename(
            defaultextension=".parm",
            filetypes=[("Parm", "*.parm"), ("所有文件", "*.*")]
        )
        if not path:
            return
        self._write_parm_file(path)
        messagebox.showinfo("导出成功", f".parm 文件已保存到:\n{path}")

    def _write_parm_file(self, path):
        lines = [
            f"# Aircraft Config: {self.name_var.get()}",
            f"# {self.desc_var.get()}",
            f"# Generated by aircraft_config_editor.py",
            "",
        ]
        param_map = {
            "physics": {},
            "aero_lift": {
                "c_lift_0": "ARSPD_LIFT_COEFF",
            },
            "flightEnvelope": {
                "airspeed_cruise": "AIRSPEED_CRUISE",
                "airspeed_max": "AIRSPEED_MAX",
                "airspeed_min": "AIRSPEED_MIN",
                "thr_max": "THR_MAX",
                "trim_throttle": "TRIM_THROTTLE",
                "ptch_lim_max": "PTCH_LIM_MAX_DEG",
                "ptch_lim_min": "PTCH_LIM_MIN_DEG",
                "roll_limit": "ROLL_LIMIT_DEG",
            },
            "pid_roll": {
                "rll_rate_p": "RLL_RATE_P",
                "rll_rate_i": "RLL_RATE_I",
                "rll_rate_d": "RLL_RATE_D",
                "rll_rate_ff": "RLL_RATE_FF",
            },
            "pid_pitch": {
                "ptch_rate_p": "PTCH_RATE_P",
                "ptch_rate_i": "PTCH_RATE_I",
                "ptch_rate_d": "PTCH_RATE_D",
                "ptch_rate_ff": "PTCH_RATE_FF",
            },
        }

        for cat, mapping in param_map.items():
            if cat in self.params:
                for key, val in self.params[cat].items():
                    if key.startswith("_"):
                        continue
                    parm_name = mapping.get(key, None)
                    if parm_name:
                        lines.append(f"{parm_name} {val['value']}")

        with open(path, "w", encoding="utf-8") as f:
            f.write("\n".join(lines) + "\n")

    def _export_to_wsl(self):
        self.params["_info"]["name"] = self.name_var.get()
        self.params["_info"]["description"] = self.desc_var.get()

        parm_path = os.path.join(os.path.dirname(__file__), "aircraft_custom.parm")
        self._write_parm_file(parm_path)

        wsl_script = f"""#!/bin/bash
# Copy custom parm to WSL
cp /mnt/d/temp/aircraft_custom.parm /tmp/ardupilot_params/aircraft.parm 2>/dev/null
cp /mnt/d/oezcon/ai_mp/aircraft_custom.parm /tmp/ardupilot_params/aircraft.parm 2>/dev/null

# Kill existing SITL
killall arduplane 2>/dev/null
sleep 2

# Start SITL with custom params
cd /tmp && /root/ardupilot/build/sitl/bin/arduplane \\
    -I0 --model plane --speedup 3 --home CMAC \\
    --defaults /root/ardupilot/Tools/autotest/models/plane.parm,/tmp/ardupilot_params/aircraft.parm \\
    2>&1 | tee /tmp/sitl_custom.log
"""
        wsl_script_path = os.path.join(os.path.dirname(__file__), "start_sitl_custom.sh")
        with open(wsl_script_path, "w") as f:
            f.write(wsl_script)

        messagebox.showinfo(
            "导出完成",
            f"文件已生成:\n{parm_path}\n{wsl_script_path}\n\n"
            "请在WSL中运行:\n"
            f"bash {wsl_script_path.replace('D:', '/mnt/d').replace('\\', '/')}"
        )


def main():
    root = tk.Tk()
    app = AircraftEditor(root)
    root.mainloop()


if __name__ == "__main__":
    main()

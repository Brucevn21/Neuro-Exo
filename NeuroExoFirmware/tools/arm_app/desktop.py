"""Windows-friendly Tk desktop interface sharing the mobile app's controller."""

from pathlib import Path
import shutil
import tkinter as tk
from tkinter import filedialog, messagebox, ttk

from scene import ArmView

BG, PANEL, TEXT, MUTED, TEAL = "#0b1523", "#112236", "#e3edf7", "#91a6bc", "#49c8b7"


class Desktop:
    def __init__(self, root, runtime, mobile_url=None):
        self.root, self.runtime = root, runtime
        self.closed = False
        root.title("NeuroExo · Arm Control")
        root.geometry("1250x860")
        root.minsize(980, 650)
        root.configure(bg=BG)
        style = ttk.Style()
        style.theme_use("clam")
        style.configure("TFrame", background=BG)
        style.configure("Card.TFrame", background=PANEL)
        style.configure("TLabel", background=BG, foreground=TEXT, font=("Segoe UI", 10))
        style.configure("Muted.TLabel", foreground=MUTED)
        style.configure("Title.TLabel", font=("Segoe UI", 24, "bold"))
        style.configure("TButton", padding=(12, 9), font=("Segoe UI", 10), background="#213b53", foreground=TEXT)
        style.map("TButton", background=[("active", "#305875"), ("disabled", "#172638")], foreground=[("disabled", "#687a8e")])
        style.configure("Primary.TButton", background="#258c80")
        style.configure("Stop.TButton", background="#ab3545", font=("Segoe UI", 13, "bold"))
        style.map("Stop.TButton", background=[("active", "#c94455")])
        style.configure("Treeview", background=PANEL, fieldbackground=PANEL, foreground=TEXT, rowheight=28)
        style.configure("Treeview.Heading", background="#213b53", foreground=TEXT)

        actionbar = ttk.Frame(root, padding=(22, 12))
        actionbar.pack(side="bottom", fill="x")
        self.stop_button = ttk.Button(actionbar, text="■  STOP", style="Stop.TButton", command=lambda: runtime.submit("stop"))
        self.stop_button.pack(side="right", padx=(15, 0))
        self.message = ttk.Label(actionbar, text="", wraplength=780)
        self.message.pack(side="left", fill="x", expand=True)
        viewport = ttk.Frame(root)
        viewport.pack(fill="both", expand=True)
        scroll_canvas = tk.Canvas(viewport, bg=BG, highlightthickness=0)
        scrollbar = ttk.Scrollbar(viewport, orient="vertical", command=scroll_canvas.yview)
        scrollbar.pack(side="right", fill="y")
        scroll_canvas.pack(side="left", fill="both", expand=True)
        scroll_canvas.configure(yscrollcommand=scrollbar.set)
        shell = ttk.Frame(scroll_canvas, padding=22)
        body = scroll_canvas.create_window((0, 0), window=shell, anchor="nw")
        shell.bind("<Configure>", lambda _: scroll_canvas.configure(scrollregion=scroll_canvas.bbox("all")))
        scroll_canvas.bind("<Configure>", lambda event: scroll_canvas.itemconfigure(body, width=event.width))
        header = ttk.Frame(shell)
        header.pack(fill="x")
        ttk.Label(header, text="NeuroExo", style="Title.TLabel").pack(side="left")
        ttk.Label(header, text="ARM CONTROL", style="Muted.TLabel").pack(side="left", padx=18)
        self.badge = ttk.Label(header, text="DISCONNECTED", style="Muted.TLabel")
        self.badge.pack(side="right")
        connection = ttk.Frame(shell, padding=(0, 16, 0, 12))
        connection.pack(fill="x")
        self.source = tk.StringVar(value="simulation")
        self.source_box = ttk.Combobox(connection, textvariable=self.source, values=["simulation", "bluetooth"], state="readonly", width=14)
        self.source_box.pack(side="left")
        self.identifier = tk.StringVar()
        ttk.Entry(connection, textvariable=self.identifier, width=23).pack(side="left", padx=8)
        ttk.Label(connection, text="BLE address optional", style="Muted.TLabel").pack(side="left")
        self.connect_button = ttk.Button(connection, text="Connect", command=self.connect)
        self.connect_button.pack(side="right")
        if mobile_url:
            ttk.Label(shell, text=f"Phone access: {mobile_url}  ·  same Wi-Fi as this computer", style="Muted.TLabel", wraplength=1100).pack(anchor="w", pady=(0, 12))

        center = ttk.Frame(shell)
        center.pack(fill="both", expand=True)
        center.columnconfigure(0, weight=3)
        center.columnconfigure(1, weight=2, minsize=325)
        center.rowconfigure(0, weight=1)
        left = ttk.Frame(center)
        left.grid(row=0, column=0, sticky="nsew", padx=(0, 20))
        canvas = tk.Canvas(left, bg=PANEL, highlightthickness=0, height=320)
        canvas.pack(fill="both", expand=True)
        self.arm = ArmView(canvas)
        metrics = ttk.Frame(left, padding=(0, 12))
        metrics.pack(fill="x")
        self.metrics = []
        for title in ("ANGLE", "MOTOR CURRENT", "PATIENT EFFORT"):
            cell = ttk.Frame(metrics)
            cell.pack(side="left", fill="x", expand=True)
            ttk.Label(cell, text=title, style="Muted.TLabel").pack(anchor="w")
            value = ttk.Label(cell, text="—", font=("Segoe UI", 20))
            value.pack(anchor="w")
            self.metrics.append(value)
        self.chart = tk.Canvas(left, bg=PANEL, highlightthickness=0, height=140)
        self.chart.pack(fill="x")

        controls = ttk.Frame(center)
        controls.grid(row=0, column=1, sticky="nsew")
        ttk.Label(controls, text="Trial setup", font=("Segoe UI", 17, "bold")).pack(anchor="w", pady=(0, 12))
        ttk.Label(controls, text="Mode").pack(anchor="w")
        self.mode = tk.StringVar(value="Assistive")
        self.mode_box = ttk.Combobox(controls, textvariable=self.mode, values=["Assistive", "Resistive", "Neutral"], state="readonly")
        self.mode_box.pack(fill="x", pady=(4, 12))
        ttk.Label(controls, text="Target angle (−150° to 80°)").pack(anchor="w")
        self.target = tk.StringVar(value="60")
        self.target_box = ttk.Spinbox(controls, from_=-150, to=80, textvariable=self.target)
        self.target_box.pack(fill="x", pady=(4, 12))
        self.level = tk.IntVar(value=5)
        self.level_label = ttk.Label(controls, text="Assistance · 50%")
        self.level_label.pack(anchor="w")
        self.slider = tk.Scale(controls, from_=1, to=9, resolution=1, orient="horizontal", variable=self.level, showvalue=False,
                               bg=BG, fg=MUTED, troughcolor="#213b53", highlightthickness=0, activebackground=TEAL,
                               command=lambda _: self.level_label.configure(text=f"{'Resistance' if self.mode.get() == 'Resistive' else 'Assistance'} · {self.level.get() * 10}%"))
        self.slider.pack(fill="x")
        ttk.Label(controls, text="10%                                      90%", style="Muted.TLabel").pack(fill="x")
        ttk.Label(controls, text="Applies to the next trial.", style="Muted.TLabel").pack(anchor="w", pady=(4, 12))
        ttk.Label(controls, text="Speed · Neutral / Demo / Home").pack(anchor="w")
        self.speed = tk.StringVar(value="Slow")
        self.speed_box = ttk.Combobox(controls, textvariable=self.speed, values=["Slow", "Medium", "High"], state="readonly")
        self.speed_box.pack(fill="x", pady=(4, 12))
        self.start_button = ttk.Button(controls, text="Start trial", style="Primary.TButton", command=lambda: self.start())
        self.start_button.pack(fill="x", pady=4)
        self.end_button = ttk.Button(controls, text="End trial & save", command=lambda: runtime.submit("stop", reason="operator_ended"))
        self.end_button.pack(fill="x", pady=4)
        modes = ttk.Frame(controls)
        modes.pack(fill="x", pady=4)
        self.demo_button = ttk.Button(modes, text="Demo", command=lambda: self.start("demo"))
        self.demo_button.pack(side="left", fill="x", expand=True, padx=(0, 4))
        self.home_button = ttk.Button(modes, text="Home", command=lambda: self.start("home"))
        self.home_button.pack(side="left", fill="x", expand=True)
        self.cal_button = ttk.Button(controls, text="Calibrate to target", command=lambda: self.start("calibrate"))
        self.cal_button.pack(fill="x", pady=4)
        self.cal_status = ttk.Label(controls, text="Calibration required", style="Muted.TLabel")
        self.cal_status.pack(anchor="w", pady=8)
        ttk.Label(controls, text="Demo moves 0° → 70° → 0°. No one attached.", wraplength=350, style="Muted.TLabel").pack(anchor="w", pady=6)

        footer = ttk.Frame(shell, padding=(0, 14, 0, 0))
        footer.pack(fill="x")
        bar = ttk.Frame(footer)
        bar.pack(fill="x")
        ttk.Label(bar, text="Saved trials", font=("Segoe UI", 12, "bold")).pack(side="left")
        self.export_button = ttk.Button(bar, text="Export selected CSV…", command=self.export)
        self.export_button.pack(side="right")
        self.table = ttk.Treeview(footer, columns=("source", "mode", "samples", "result"), show="headings", height=3, selectmode="browse")
        for name, width in (("source", 120), ("mode", 120), ("samples", 90), ("result", 360)):
            self.table.heading(name, text=name.title())
            self.table.column(name, width=width)
        self.table.pack(fill="x", pady=(8, 0))

        def scroll_body(event):
            if event.widget not in (canvas, self.slider):
                direction = -1 if getattr(event, "num", None) == 4 or getattr(event, "delta", 0) > 0 else 1
                scroll_canvas.yview_scroll(direction * 3, "units")

        root.bind("<MouseWheel>", scroll_body)
        root.bind("<Button-4>", scroll_body)
        root.bind("<Button-5>", scroll_body)
        self.poll()

    def connect(self):
        state = self.runtime.snapshot()
        if state["connected"]:
            self.runtime.submit("disconnect")
        else:
            self.runtime.submit("connect", source=self.source.get(), identifier=self.identifier.get())

    def start(self, mode=None):
        mode = mode or self.mode.get().lower()
        try:
            target = int(self.target.get()) if mode not in ("demo", "home") else 0
            setting = self.level.get() if mode in ("assistive", "resistive") else (0 if mode == "calibrate" else ["Slow", "Medium", "High"].index(self.speed.get()))
        except ValueError:
            messagebox.showerror("Target angle", "Enter a whole number from −150 to 80 degrees.")
            return
        self.runtime.submit("start", mode=mode, setting=setting, target=target)

    def export(self):
        selected = self.table.selection()
        if not selected:
            return
        trial = next(t for t in self.runtime.snapshot()["trials"] if t["id"] == selected[0])
        destination = filedialog.asksaveasfilename(defaultextension=".csv", initialfile=trial["file"], filetypes=[("Trial CSV", "*.csv")])
        if destination:
            try:
                source = self.runtime.controller.log_dir / trial["file"]
                if source.resolve() != Path(destination).resolve():
                    shutil.copyfile(source, destination)
            except OSError as exc:
                messagebox.showerror("Export failed", str(exc))

    def poll(self):
        if self.closed:
            return
        state = self.runtime.snapshot()
        if state["connected"] or state["connecting"]:
            self.source.set(state["source"])
        self.badge.configure(text=("SIMULATOR" if state["source"] == "simulation" else "BLUETOOTH") + (" · LIVE" if state["fresh"] else " · OFFLINE"))
        self.connect_button.configure(text="Disconnect" if state["connected"] else "Connect", state="disabled" if state["connecting"] else "normal")
        self.source_box.configure(state="disabled" if state["connected"] or state["connecting"] else "readonly")
        busy = state["active"] or state["pending"] is not None
        for widget in (self.mode_box, self.speed_box):
            widget.configure(state="disabled" if busy else "readonly")
        self.target_box.configure(state="disabled" if busy else "normal")
        self.slider.configure(state="disabled" if busy or self.mode.get() == "Neutral" else "normal")
        self.level_label.configure(text=f"{'Resistance' if self.mode.get() == 'Resistive' else 'Assistance'} · {self.level.get() * 10}%")
        ready = state["can_start"]
        self.start_button.configure(state="normal" if ready and (self.mode.get() == "Neutral" or state["calibrated"]) else "disabled")
        for widget in (self.demo_button, self.home_button, self.cal_button):
            widget.configure(state="normal" if ready else "disabled")
        self.end_button.configure(state="normal" if state["connected"] and busy else "disabled")
        self.stop_button.configure(state="normal" if state["connected"] else "disabled")
        self.cal_status.configure(text="Calibrated · range must cover trial" if state["calibrated"] else "Calibration required for Assistive / Resistive")
        for label, value in zip(self.metrics, (f"{state['angle']:.1f}°", f"{state['current_a']:.3f} A", f"{state['effort_nm']:.2f} N·m")):
            label.configure(text=value if state["fresh"] else "—")
        self.message.configure(text=state["message"])
        self.arm.draw(state["angle"], state["target"])
        self.draw_chart(state["history"])
        for trial in reversed(state["trials"]):
            if not self.table.exists(trial["id"]):
                self.table.insert("", 0, iid=trial["id"], values=(trial["source"], trial["mode"], trial["samples"], trial["result"].replace("_", " ")))
                self.table.selection_set(trial["id"])
        self.export_button.configure(state="normal" if state["trials"] else "disabled")
        self.root.after(100, self.poll)

    def draw_chart(self, history):
        c = self.chart
        c.delete("all")
        w, h = c.winfo_width(), c.winfo_height()
        c.create_text(12, 12, text="ANGLE / LAST 1,500 SAMPLES", fill=MUTED, anchor="nw", font=("Segoe UI", 8))
        if len(history) < 2 or w < 80:
            c.create_text(w / 2, h / 2, text="Trial angle history appears here", fill=MUTED)
            return
        low, high = min(p[1] for p in history) - 5, max(p[1] for p in history) + 5
        duration = max(1, history[-1][0] - history[0][0])
        points = [(42 + (p[0] - history[0][0]) / duration * (w - 58), h - 22 - (p[1] - low) / (high - low) * (h - 55)) for p in history]
        c.create_line(*[v for point in points for v in point], fill=TEAL, width=2)
        c.create_text(8, 36, text=f"{high:.0f}°", fill=MUTED, anchor="w")
        c.create_text(8, h - 24, text=f"{low:.0f}°", fill=MUTED, anchor="w")
        c.create_text(w - 10, h - 8, text=f"{history[-1][0]:.1f} s", fill=MUTED, anchor="e")

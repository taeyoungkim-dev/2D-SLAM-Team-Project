import 'package:flutter/material.dart';
import 'package:provider/provider.dart';

import '../core/app_config.dart';
import '../state/robot_controller.dart';
import '../theme/cupertino_style.dart';

/// Local Backend Server(FastAPI)의 IP/포트를 설정하는 화면.
///
/// 데모 당일 로봇/서버 PC의 LAN IP가 매번 바뀔 수 있으므로, 코드를 다시
/// 빌드하지 않고 앱에서 직접 바꿀 수 있게 한다.
class SettingsScreen extends StatefulWidget {
  const SettingsScreen({super.key});

  @override
  State<SettingsScreen> createState() => _SettingsScreenState();
}

class _SettingsScreenState extends State<SettingsScreen> {
  late final TextEditingController _hostController;
  late final TextEditingController _portController;
  bool _testing = false;
  String? _testResult;

  @override
  void initState() {
    super.initState();
    final config = context.read<AppConfig>();
    _hostController = TextEditingController(text: config.host);
    _portController = TextEditingController(text: config.port.toString());
  }

  @override
  void dispose() {
    _hostController.dispose();
    _portController.dispose();
    super.dispose();
  }

  Future<void> _save() async {
    final config = context.read<AppConfig>();
    final port = int.tryParse(_portController.text.trim()) ?? config.port;
    await config.update(host: _hostController.text.trim(), port: port);
    if (mounted) {
      showAppSnackBar(
        context,
        child: const GlassPanel(
          dark: true,
          radius: 20,
          padding: EdgeInsets.symmetric(horizontal: 16, vertical: 14),
          child: Row(
            children: [
              Icon(CupertinoIcons.checkmark_circle, size: 20, color: Colors.white),
              SizedBox(width: 10),
              Expanded(
                child: Text(
                  '서버 설정을 저장했습니다.',
                  style: TextStyle(
                    color: Colors.white,
                    fontSize: 14,
                    fontWeight: FontWeight.w500,
                  ),
                ),
              ),
            ],
          ),
        ),
      );
    }
  }

  Future<void> _testConnection() async {
    setState(() {
      _testing = true;
      _testResult = null;
    });
    final controller = context.read<RobotController>();
    final ok = await controller.api.ping();
    setState(() {
      _testing = false;
      _testResult = ok ? '연결 성공' : '연결 실패';
    });
  }

  @override
  Widget build(BuildContext context) {
    return Scaffold(
      extendBodyBehindAppBar: true,
      appBar: const AppTopBar(title: '서버 설정'),
      body: SingleChildScrollView(
        physics: const BouncingScrollPhysics(),
        padding: EdgeInsets.fromLTRB(
          16,
          MediaQuery.paddingOf(context).top + 16,
          16,
          MediaQuery.paddingOf(context).bottom + 24,
        ),
        child: SpringEntrance(
          child: Column(
            crossAxisAlignment: CrossAxisAlignment.stretch,
            children: [
              AppCard(
                radius: 26,
                padding: const EdgeInsets.all(18),
                child: Column(
                  crossAxisAlignment: CrossAxisAlignment.stretch,
                  children: [
                    const Text(
                      'Local Backend Server(FastAPI)가 실행 중인 PC의 로컬 IP와 포트를 '
                      '입력하세요. (예: 192.168.0.10 / 8000)',
                      style: AppText.callout,
                    ),
                    const SizedBox(height: 16),
                    _SettingsField(
                      controller: _hostController,
                      label: '서버 IP',
                    ),
                    const SizedBox(height: 12),
                    _SettingsField(
                      controller: _portController,
                      label: '포트',
                    ),
                  ],
                ),
              ),
              const SizedBox(height: 20),
              AppButton(label: '저장', onPressed: _save),
              const SizedBox(height: 10),
              AppButton(
                variant: AppButtonVariant.outlined,
                label: '연결 테스트',
                loading: _testing,
                onPressed: _testing ? null : _testConnection,
              ),
              AnimatedSize(
                duration: const Duration(milliseconds: 350),
                curve: Curves.easeOutCubic,
                alignment: Alignment.topCenter,
                child: AnimatedSwitcher(
                  duration: AppSprings.bouncy.duration,
                  transitionBuilder: springSwitcherTransition,
                  child: _testResult == null
                      ? const SizedBox(width: double.infinity)
                      : Padding(
                          key: ValueKey(_testResult),
                          padding: const EdgeInsets.only(top: 14),
                          child: Center(
                            child: AppChip(
                              icon: _testResult == '연결 성공'
                                  ? CupertinoIcons.checkmark_circle
                                  : CupertinoIcons.exclamationmark_circle,
                              label: _testResult!,
                              inverted: _testResult == '연결 성공',
                            ),
                          ),
                        ),
                ),
              ),
            ],
          ),
        ),
      ),
    );
  }
}

/// Squircle 배경의 라벨 텍스트 필드.
class _SettingsField extends StatelessWidget {
  const _SettingsField({required this.controller, required this.label});

  final TextEditingController controller;
  final String label;

  @override
  Widget build(BuildContext context) {
    return DecoratedBox(
      decoration: ShapeDecoration(
        color: AppColors.background,
        shape: squircle(16),
      ),
      child: TextField(
        controller: controller,
        keyboardType: TextInputType.number,
        style: AppText.body,
        decoration: InputDecoration(
          labelText: label,
          labelStyle: AppText.callout,
          border: InputBorder.none,
          contentPadding:
              const EdgeInsets.symmetric(horizontal: 16, vertical: 14),
        ),
      ),
    );
  }
}
